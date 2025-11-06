#!/usr/bin/env python3
import os, sys, argparse, random, time, select, tty, termios

from find_carla_egg import find_carla_egg
carla_egg_file = find_carla_egg()
sys.path.append(carla_egg_file)
sys.stdout.reconfigure(line_buffering=True)
import carla

# Colors
red    = carla.Color(255,   0,   0)
green  = carla.Color(  0, 255,   0)
blue   = carla.Color( 47, 210, 231)
cyan   = carla.Color(  0, 255, 255)
yellow = carla.Color(255, 255,   0)
orange = carla.Color(255, 162,   0)
white  = carla.Color(255, 255, 255)

trail_life_time = 10
waypoint_separation = 2  # meters

# ---------- Drawing helpers ----------
def draw_transform(debug, trans, col=white, lt=-1):
    debug.draw_arrow(
        trans.location,
        trans.location + trans.get_forward_vector(),
        thickness=0.05, arrow_size=0.1, color=col, life_time=lt)

def draw_waypoint_union(debug, w0, w1, color=green, lt=5):
    debug.draw_line(
        w0.transform.location + carla.Location(z=0.25),
        w1.transform.location + carla.Location(z=0.25),
        thickness=0.1, color=color, life_time=lt, persistent_lines=False)
    debug.draw_point(w1.transform.location + carla.Location(z=0.25), 0.1, color, lt, False)

def draw_waypoint_info(debug, w, lt=5):
    w_loc = w.transform.location
    debug.draw_string(w_loc + carla.Location(z=0.5),  f"lane: {w.lane_id}", False, yellow, lt)
    debug.draw_string(w_loc + carla.Location(z=1.0),  f"road: {w.road_id}", False, blue,   lt)
    debug.draw_string(w_loc + carla.Location(z=-0.5), f"{w.lane_change}",    False, red,    lt)

def draw_junction(debug, junction, l_time=10):
    box = junction.bounding_box
    p1 = box.location + carla.Location(x= box.extent.x, y= box.extent.y, z=2)
    p2 = box.location + carla.Location(x=-box.extent.x, y= box.extent.y, z=2)
    p3 = box.location + carla.Location(x=-box.extent.x, y=-box.extent.y, z=2)
    p4 = box.location + carla.Location(x= box.extent.x, y=-box.extent.y, z=2)
    # for a,b in [(p1,p2),(p2,p3),(p3,p4),(p4,p1)]:
    #     debug.draw_line(a,b, 0.1, orange, l_time, False)
    # for pa, pb in junction.get_waypoints(carla.LaneType.Any):
    #     draw_transform(debug, pa.transform, orange, l_time)
    #     debug.draw_point(pa.transform.location + carla.Location(z=0.75), 0.1, orange, l_time, False)
    #     draw_transform(debug, pb.transform, orange, l_time)
    #     debug.draw_point(pb.transform.location + carla.Location(z=0.75), 0.1, orange, l_time, False)
        # debug.draw_line(pa.transform.location + carla.Location(z=0.75),
        #                 pb.transform.location + carla.Location(z=0.75),
        #                 0.1, white, l_time, False)

def draw_choice_list(debug, current_w, choices, selected_idx, direction_label, lt=trail_life_time):
    """
    Render choices; for each choice 'w', draw ONLY the junction lane segment connected to that choice
    by matching road/lane/section IDs. Selected = green; others = orange.
    """
    if not choices:
        return

    for i, w in enumerate(choices):
        col = green if i == selected_idx else orange

        # Prefer the junction associated with the choice waypoint; fall back to current waypoint's junction
        junction = None
        if getattr(w, "is_junction", False):
            try:
                junction = w.get_junction()
            except RuntimeError:
                junction = None
        if junction is None and getattr(current_w, "is_junction", False):
            try:
                junction = current_w.get_junction()
            except RuntimeError:
                junction = None

        # Always show the label at the choice point
        label_loc = w.transform.location + carla.Location(z=0.8)
        debug.draw_string(label_loc, f"{direction_label} [{i}]", False, col, lt)

        if junction is None:
            # Not in a junction context; nothing to draw for junction lanes
            continue

        # IDs of the choice waypoint
        w_r = w.road_id
        w_l = w.lane_id
        w_sct = getattr(w, "section_id", None)

        # Find and draw exactly the lane segment from junction that connects to this choice:
        # - if w is inside the junction, match w to 'pa' (segment start inside junction)
        # - otherwise, match w to 'pb' (segment end on the outgoing road)
        match_drawn = False
        for pa, pb in junction.get_waypoints(carla.LaneType.Driving):
            if w.is_junction:
                # Match on the BEGIN waypoint 'pa'
                if (pa.road_id == w_r and pa.lane_id == w_l and
                    (getattr(pa, "section_id", None) == w_sct)):
                    debug.draw_line(
                        pa.transform.location + carla.Location(z=0.75),
                        pb.transform.location + carla.Location(z=0.75),
                        thickness=0.1,
                        color=col,
                        life_time=lt,
                        persistent_lines=False
                    )
                    match_drawn = True
                    break
            else:
                # Match on the END waypoint 'pb'
                if (pb.road_id == w_r and pb.lane_id == w_l and
                    (getattr(pb, "section_id", None) == w_sct)):
                    debug.draw_line(
                        pa.transform.location + carla.Location(z=0.75),
                        pb.transform.location + carla.Location(z=0.75),
                        thickness=0.1,
                        color=col,
                        life_time=lt,
                        persistent_lines=False
                    )
                    match_drawn = True
                    break

        # If section_id occasionally misaligns between choice and (pa/pb), you can relax it:
        #   ...and (getattr(pa, "section_id", None) == w_sct)
        # → replace with just road+lane equality.
        # This keeps it purely ID-based without falling back to distances.


# ---------- Candidate selection preferences ----------
def select_forward_candidate(curr, cands):
    same = [w for w in cands if w.road_id == curr.road_id and w.lane_id == curr.lane_id]
    if same:
        return same[0]
    same_lane = [w for w in cands if w.lane_id == curr.lane_id]
    if same_lane:
        return same_lane[0]
    return cands[0] if cands else None

def select_backward_candidate(curr, cands):
    same = [w for w in cands if w.road_id == curr.road_id and w.lane_id == curr.lane_id]
    if same:
        return same[0]
    same_lane = [w for w in cands if w.lane_id == curr.lane_id]
    if same_lane:
        return same_lane[0]
    return cands[0] if cands else None

# ---------- Terminal key reader with arrow support ----------
class KeyReader:
    """
    Non-blocking single-key reader that recognizes arrow keys.
    Returns: 'w','s','q', 'LEFT','RIGHT','UP','DOWN', or '' when idle.
    """
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self
    def __exit__(self, et, ev, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
    def _read_available(self, n, timeout):
        # read up to n chars if available
        out = ''
        end = time.time() + timeout
        while len(out) < n and time.time() < end:
            r,_,_ = select.select([sys.stdin], [], [], max(0.0, end - time.time()))
            if r:
                out += sys.stdin.read(1)
            else:
                break
        return out
    def readkey(self, timeout=0.1):
        r,_,_ = select.select([sys.stdin], [], [], timeout)
        if not r:
            return ''
        ch = sys.stdin.read(1)
        if ch == '\x1b':  # ESC sequence
            seq = ch + self._read_available(2, 0.01)  # typically ESC [ A/B/C/D
            if seq.startswith('\x1b['):
                code = seq[2:3]
                if code == 'A': return 'UP'
                if code == 'B': return 'DOWN'
                if code == 'C': return 'RIGHT'
                if code == 'D': return 'LEFT'
            return ''  # unrecognized escape
        return ch

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('-p','--port', default=2000, type=int)
    ap.add_argument('-i','--info', action='store_true', help='Draw lane/road/lane_change text')
    ap.add_argument('--manual', action='store_true', help='Manual crawl with branch selection')
    ap.add_argument('-x', default=0.0, type=float)
    ap.add_argument('-y', default=0.0, type=float)
    ap.add_argument('-z', default=0.0, type=float)
    ap.add_argument('-s','--seed', default=os.getpid(), type=int)
    ap.add_argument('-t','--tick-time', default=0.2, type=float, help='Random mode tick time')
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)

    world = client.get_world()
    m = world.get_map()
    debug = world.debug

    random.seed(args.seed)
    print("Seed:", args.seed)
    loc = carla.Location(args.x, args.y, args.z)
    print("Initial location:", loc)

    current_w = m.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)

    # Selection UI state
    selecting = False
    select_mode = None            # 'forward' or 'backward'
    choices = []
    sel_idx = 0

    if args.manual:
        print("Manual mode:")
        print("  w = step forward (if multiple branches: enter selection; use q/e to choose, press w to confirm)")
        print("  s = step backward (same selection behavior)")
        print("  Ctrl + C = quit")
        with KeyReader() as kr:
            while True:
                draw_transform(debug, current_w.transform, white, trail_life_time)
                if args.info:
                    draw_waypoint_info(debug, current_w, trail_life_time)

                # Render selection overlays if active
                if selecting:
                    direction_label = 'FWD' if select_mode == 'forward' else 'BACK'
                    draw_choice_list(debug, current_w, choices, sel_idx, direction_label, trail_life_time)

                key = kr.readkey(timeout=0.1)
                if not key:
                    continue

                if key in ('\x03', '\x04'):
                    break
                # print(f'Key pressed: {key}', flush=True)
                # Cycle choices if in selection state
                if selecting and key in ('q','e'):
                    
                    if not choices:
                        selecting = False
                        continue
                    if key == 'q':
                        print(f'Switching selection left before: {sel_idx}')
                        sel_idx = (sel_idx - 1) % len(choices)
                        print(f'Switching selection left after: {sel_idx}')
                    elif key == 'e':  # RIGHT
                        print(f'Switching selection right before: {sel_idx}')
                        sel_idx = (sel_idx + 1) % len(choices)
                        print(f'Switching selection right before: {sel_idx}')
                    # re-draw highlighting
                    direction_label = 'FWD' if select_mode == 'forward' else 'BACK'
                    draw_choice_list(debug, current_w, choices, sel_idx, direction_label, trail_life_time)
                    continue

                # Confirm selection with w/s depending on mode
                if selecting and ((select_mode == 'forward' and key.lower() == 'w') or
                                  (select_mode == 'backward' and key.lower() == 's')):
                    chosen = choices[sel_idx]
                    # visualize chosen
                    draw_waypoint_union(debug, current_w, chosen, cyan if select_mode=='forward' else red, trail_life_time)
                    current_w = chosen
                    if current_w.is_junction:
                        draw_junction(debug, current_w.get_junction(), trail_life_time)
                    # exit selection mode
                    selecting = False
                    choices = []
                    sel_idx = 0
                    select_mode = None
                    continue

                # Enter selection or auto-step for forward/backward
                if key.lower() == 'w':
                    cands = list(current_w.next(waypoint_separation))
                    if not cands:
                        print("[warn] no forward waypoint from here")
                        continue
                    if len(cands) == 1:
                        nxt = cands[0]
                        draw_waypoint_union(debug, current_w, nxt, green, trail_life_time)
                        current_w = nxt
                        if current_w.is_junction:
                            draw_junction(debug, current_w.get_junction(), trail_life_time)
                    else:
                        # stop and present choices; do not move yet
                        selecting = True
                        select_mode = 'forward'
                        choices = cands
                        # pick a sane default highlight (prefer same road/lane)
                        default = select_forward_candidate(current_w, cands)
                        sel_idx = cands.index(default) if default in cands else 0
                        draw_choice_list(debug, current_w, choices, sel_idx, 'FWD', trail_life_time)
                    continue

                if key.lower() == 's':
                    cands = list(current_w.previous(waypoint_separation))
                    if not cands:
                        print("[warn] no backward waypoint from here")
                        continue
                    if len(cands) == 1:
                        prv = cands[0]
                        draw_waypoint_union(debug, prv, current_w, red, trail_life_time)
                        current_w = prv
                    else:
                        selecting = True
                        select_mode = 'backward'
                        choices = cands
                        default = select_backward_candidate(current_w, cands)
                        sel_idx = cands.index(default) if default in cands else 0
                        draw_choice_list(debug, current_w, choices, sel_idx, 'BACK', trail_life_time)
                    continue

                # Ignore other keys for now (we’ll wire A/D, arrows-as-lane-change later)

    else:
        # Random crawl unchanged (still here for completeness)
        while True:
            potential_w = list(current_w.next(waypoint_separation))
            if current_w.lane_change & carla.LaneChange.Right:
                right_w = current_w.get_right_lane()
                if right_w and right_w.lane_type == carla.LaneType.Driving:
                    potential_w += list(right_w.next(waypoint_separation))
            if current_w.lane_change & carla.LaneChange.Left:
                left_w = current_w.get_left_lane()
                if left_w and left_w.lane_type == carla.LaneType.Driving:
                    potential_w += list(left_w.next(waypoint_separation))

            if potential_w:
                fwd_pref = select_forward_candidate(current_w, potential_w)
                next_w = fwd_pref if fwd_pref else random.choice(potential_w)
                if next_w in potential_w:
                    potential_w.remove(next_w)
            else:
                print("[warn] dead-end: no next waypoints; trying previous()")
                prevs = list(current_w.previous(waypoint_separation))
                if not prevs:
                    print("[fatal] nowhere to go from here.")
                    break
                next_w = select_backward_candidate(current_w, prevs)

            draw_waypoint_union(debug, current_w, next_w, cyan if current_w.is_junction else green, trail_life_time)
            draw_transform(debug, current_w.transform, white, trail_life_time)
            for p in potential_w:
                draw_waypoint_union(debug, current_w, p, red, trail_life_time)
                draw_transform(debug, p.transform, white, trail_life_time)
            if next_w.is_junction:
                draw_junction(debug, next_w.get_junction(), trail_life_time)

            current_w = next_w
            time.sleep(args.tick_time)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExit by user.")
    finally:
        print("\nExit.")

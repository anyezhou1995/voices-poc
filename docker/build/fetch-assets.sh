#!/usr/bin/env bash
set -euo pipefail

# Usage: ./fetch-assets.sh [path/to/env]
# Optional: STRICT_MATCH=0 to warn instead of exit when no files match the pattern
: "${STRICT_MATCH:=1}"

# Prompt for env file if no arg; enable tab completion
if [[ $# -eq 0 ]]; then
  read -e -p "Enter path to env file: " ENV_FILE
else
  ENV_FILE="$1"
fi

if [[ ! -f "$ENV_FILE" ]]; then
  echo "env file not found: $ENV_FILE" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$ENV_FILE"

if [[ -z "${BUILD_DIR:-}" ]]; then
  echo "BUILD_DIR not set in env file" >&2
  exit 1
fi

mkdir -p "$BUILD_DIR"

# ---- gdown bootstrap (module form; avoids broken shebangs) ----
export PATH="$HOME/.local/bin:$PATH"
GDOWN="python3 -m gdown"
if ! python3 - <<'PY' >/dev/null 2>&1
import importlib; importlib.import_module("gdown")
PY
then
  echo "Installing gdown for current Python..."
  if command -v pip3 >/dev/null 2>&1; then
    pip3 install --user -q gdown
  elif python3 -m ensurepip --upgrade --user >/dev/null 2>&1; then
    python3 -m pip install --user -q gdown
  else
    echo "pip3 is not available and ensurepip failed. Please install pip." >&2
    exit 1
  fi
  python3 - <<'PY' || { echo "gdown install failed"; exit 1; }
import importlib; importlib.import_module("gdown")
PY
fi
# ---------------------------------------------------------------

check_pattern_match() {
  # Ensure at least one file in BUILD_DIR matches the pattern
  # STRICT_MATCH=1 -> exit on failure, 0 -> warn only
  local key="$1"
  local pattern="$2"

  shopt -s nullglob
  # unquoted to allow glob expansion intentionally
  # shellcheck disable=SC2086
  local matches=( $BUILD_DIR/$pattern )
  shopt -u nullglob

  if (( ${#matches[@]} == 0 )); then
    echo "! $key: download finished but no files match pattern '$pattern' in $BUILD_DIR" >&2
    if [[ "$STRICT_MATCH" -eq 1 ]]; then
      exit 1
    else
      return 0
    fi
  fi

  echo "? $key: ${#matches[@]} file(s) match pattern '$pattern'"
}

download_one() {
  local key="$1"
  local glob_var="${key}_GLOB"
  local id_var="${key}_ID"
  local out_var="${key}_OUT"   # optional explicit output name

  # indirect expansion
  local pattern="${!glob_var:-}"
  local id="${!id_var:-}"
  local out="${!out_var:-}"

  if [[ -z "$pattern" || -z "$id" ]]; then
    echo "skipping $key ? missing ${glob_var} or ${id_var} in env" >&2
    return
  fi

  shopt -s nullglob
  # shellcheck disable=SC2086
  local matches=( $BUILD_DIR/$pattern )
  shopt -u nullglob

  if (( ${#matches[@]} > 0 )); then
    echo "? $key: found ${#matches[@]} file(s) matching '$pattern'; skipping download"
    return
  fi

  # If OUT is set but does not match the pattern, warn early
  if [[ -n "$out" && ! "$out" == $pattern ]]; then
    echo "! $key: OUT '$out' does not match pattern '$pattern' (will still download)" >&2
  fi

  echo "? $key: no files matching '$pattern' in $BUILD_DIR; downloading?"
  if [[ -n "$out" ]]; then
    $GDOWN "$id" -O "$BUILD_DIR/$out"
  else
    ( cd "$BUILD_DIR" && $GDOWN "$id" )
  fi
  echo "? $key: download complete"

  # Post-download verification
  check_pattern_match "$key" "$pattern"
}

for key in ${ASSETS:-}; do
  download_one "$key"
done

echo "all done. files in: $BUILD_DIR"

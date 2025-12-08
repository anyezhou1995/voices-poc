#!/bin/bash

echo "Installing Distributed Testing software..."

# Get the directory of the script, no matter where it's called from
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DT_SCRIPT="$SCRIPT_DIR/dt"
DT_AUTOCOMPLETE="$SCRIPT_DIR/__dt_autocomplete"

VUG_LOCAL_DT_PATH=$(echo "$SCRIPT_DIR" | sed -E 's|(/distributed-testing).*|\1|')

# Ensure required files exist
if [[ ! -f "$DT_SCRIPT" || ! -f "$DT_AUTOCOMPLETE" ]]; then
    echo "Error: Required files dt or __dt_autocomplete not found in $SCRIPT_DIR"
    exit 1
fi

# Check for existing dt in /usr/bin
if [[ -f /usr/bin/dt ]]; then
    if grep -qi "distributed testing" /usr/bin/dt; then
        echo "A previous version of the Distributed Testing script was found."
        read -p "Would you like to overwrite it? [y/N] " confirm
        if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
            echo "Installation aborted."
            exit 1
        fi
    else
        echo "Another application is using the name 'dt' in /usr/bin."
        echo "Please remove it manually before installing this script."
        exit 1
    fi
fi

# Install dt script
sudo cp "$DT_SCRIPT" /usr/bin/dt
sudo chmod ugo+x /usr/bin/dt

# Install autocomplete
sudo cp "$DT_AUTOCOMPLETE" /etc/bash_completion.d/__dt_autocomplete
sudo chmod ugo+x /etc/bash_completion.d/__dt_autocomplete

# remove old and reset autocompletion
complete -r dt
source /etc/bash_completion.d/__dt_autocomplete

# Remove old config from previous versions
old_config_found=0

# Check for source lines
grep -q 'source ~/.voices_site_config' ~/.bashrc && old_config_found=1
grep -q 'source ~/.voices_scenario_config' ~/.bashrc && old_config_found=1

# Check for files
[ -f ~/.voices_site_config ] && old_config_found=1
[ -f ~/.voices_scenario_config ] && old_config_found=1

if [ $old_config_found -eq 1 ]; then
    echo "Symbolic links from a previous version of distributed testing detected."
    echo "These links have been renamed."
    read -p "Do you want to remove the old ones from your system? (y/n): " yn
    if [[ "$yn" == [Yy]* ]]; then
        sed -i '/source ~\/\.voices_site_config/d' ~/.bashrc
        sed -i '/source ~\/\.voices_scenario_config/d' ~/.bashrc
        rm -f ~/.voices_site_config ~/.voices_scenario_config
        echo "All old configuration removed."
    else
        echo "Nothing removed."
    fi
fi

dt init $VUG_LOCAL_DT_PATH

# Create logs directory 

mkdir -p $VUG_LOCAL_DT_PATH/logs

echo "Setting permissions for log directory"

sudo chmod -R a+rw $VUG_LOCAL_DT_PATH/logs

echo "Installation complete. You may need to restart your terminal for autocomplete to take effect."

#!/bin/bash

# This script reads a path from the user, validates it, and returns the absolute path
# Returns the absolute path on stdout
# All other diagnostic messages go to stderr

# exit if any command below fails
set -e
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m'

# Message handling functions
msg() { echo "$@" >&2; }
info() { echo -e "${GREEN}$@${NC}" >&2; }
warn() { echo -e "${YELLOW}$@${NC}" >&2; }
error() { echo -e "${RED}$@${NC}" >&2; }
prompt() { echo -n "$@ " >&2; }

# Get absolute path
get_abs_path() {
  # $1 : directory path
  echo "$(cd "$1" 2>/dev/null && pwd || echo "")"
}

# Function to handle operations requiring sudo
# Parameters:
#   $1: Directory to check for write permissions
#   $2: Command to run
#   $3: Success message
#   $4: Error message if sudo denied
run_with_sudo() {
    local check_dir="$1"
    local command="$2"
    local success_msg="$3"
    local error_msg="$4"
    
    if [ ! -w "$check_dir" ]; then
        warn "Warning: You don't have write permission to $check_dir"
        msg "This operation requires root privileges"
        prompt "Continue with sudo? (Y/n):"
        read use_sudo
        
        if [[ "$use_sudo" == "y" || "$use_sudo" == "Y" ]]; then
            if sudo bash -c "$command"; then
                info "$success_msg"
            else
                error "Command failed with sudo"
                msg "Abort"
                exit 0
            fi
        else
            error "Unable to get path, $error_msg"
            msg "Abort"
            exit 0
        fi
    else
        bash -c "$command"
        info "$success_msg"
    fi
}

# Prompt for parent directory
prompt "Enter the workspace directory path (leave empty for default $HOME/):"
read parent_dir

# Set to home directory if empty
if [ -z "$parent_dir" ]; then
    msg "Setting workspace directory path to $HOME/"
    parent_dir=~
else
    # Check if the path is relative
    if [[ ! "$parent_dir" = /* ]]; then
        msg "Using relative path from home directory"
        parent_dir="$HOME/$parent_dir"
    fi
    # Check if the parent directory exists
    if [ ! -d "$parent_dir" ]; then
        prompt "Parent directory $parent_dir doesn't exist. Create it? (Y/n):"
        read create_parent
        if [[ "$create_parent" == "y" || "$create_parent" == "Y" ]]; then
            # Check if we need root privileges to create the parent
            parent_of_dir=$(dirname "$parent_dir")
            
            # Create directory and make it writable by current user if needed
            run_with_sudo "$parent_of_dir" \
                "mkdir -p \"$parent_dir\" && chown $USER:$(id -gn) \"$parent_dir\"" \
                "Parent directory created successfully and permissions set" \
                "cannot create directory"
        else
            msg "Parent directory not created"
            msg "Abort"
            exit 0
        fi
    elif [ ! -w "$parent_dir" ]; then
        # Parent exists but isn't writable
        run_with_sudo "$parent_dir" \
            "chmod u+w \"$parent_dir\"" \
            "Directory permissions updated successfully" \
            "cannot modify permissions"
    fi
fi

# Get absolute path after ensuring directory exists
parent_dir=$(get_abs_path "$parent_dir")
if [ -z "$parent_dir" ]; then
    error "Failed to get absolute path"
    exit 0
fi

# Only output the path to stdout for capture
echo "$parent_dir"
exit 0
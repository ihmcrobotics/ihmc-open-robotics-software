#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

# Check if the username argument is provided
if [ $# -ne 1 ]; then
    echo "Error: Please provide the username as an argument."
    echo "Format: First letter of first name and last name, all lowercase (e.g., rrobot for Rosie Robot)."
    echo "Usage: $0 <username>"
    exit 1
fi

USERNAME="$1"

scp -r "$PWD" "$USERNAME"@gpu2.ihmc.us:/home/"$USERNAME"/

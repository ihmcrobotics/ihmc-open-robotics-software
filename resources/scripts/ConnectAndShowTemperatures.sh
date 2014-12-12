#!/bin/bash
# Connect to Valkyrie and run the show_temperatures.sh script.

ssh -X robonaut@139.169.44.23 "cd /home/robonaut/startup; ./show_temperatures.sh\r"
echo "***REMOVED***\r"
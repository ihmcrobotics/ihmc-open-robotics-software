#!/bin/bash
# Connect to Valkyrie and run the coeff_editor.sh script.

ssh -X robonaut@139.169.44.23 "cd /home/robonaut/startup; rosrun nasa_configuration coeff_editor.py\r"
echo "***REMOVED***\r"
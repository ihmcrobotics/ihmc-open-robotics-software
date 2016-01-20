#!/bin/bash
# Connect to Valkyrie and run the show_temperatures.sh script.

ssh -X -t -t robonaut@139.169.44.23 << _EOF_
cd /home/robonaut/startup; ./show_temperatures.sh
_EOF_


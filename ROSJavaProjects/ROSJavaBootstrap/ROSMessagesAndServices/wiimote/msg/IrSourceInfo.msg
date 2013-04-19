# Sensor data pertaining to the Wiimote infrared camera.
# This message contains data for one of the four infrared 
# light sources that the camera detects.
#
# Each light is specified with a 2D position and 
# a 'source magnitude' (ir_size). If the x dimension
# is set to INVALID_FLOAT, then no light was detected for 
# the respective light. The Wiimote handles up to
# four light sources, and the wiimote_node.py software
# is written to that limit as well.
#
# I am unsure what the 'ir_size' values represent. 
# They are described as 'source magnitude' in some places. I
# *assume* this is signal amplitude, but it's unclear. 
# Note that current lowest level cwiid driver does not 
# seem to pass the ir_size value to the cwiid Wiimote.c. 
# For now this size will therefore be set INVALID

float64 x 
float64 y 
int64 ir_size

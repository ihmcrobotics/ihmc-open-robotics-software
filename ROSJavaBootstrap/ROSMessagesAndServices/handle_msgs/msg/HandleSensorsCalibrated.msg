# This is sensors of the HANDLE hand after calibration and data manipulation
# published from the package sensors, by the sensors_publisher

# not all the sensors were included, but only the one which were addressed at the moment

# Currently only used for time stamp.  
Header header

# The tactile array for each finger.  In units of kPa.
# [F1, F2, F3]
# Note there are 12 proximal and 10 distal sensors.
Finger[3] fingerTactile

# The tactile array for the palm.  In units of kPa.
float32[48] palmTactile

# The encoder on the F1 / F2 rotation.
# Approx. 768 ticks to rotate the fingers 90 degrees.
float32 fingerSpread

# The proximal joint angle. Angle in radians
# [F1, F2, F3]
float32[3] proximalJointAngle

# The finger distal joint flexture angle
# [F1, F2, F3]
# Note there are 4 readings on either side of the joint.
Finger[3] distalJointAngle


## ComHeightPacketMessage
# This message sets the robot's center of mass height.

float64 MIN_COM_HEIGHT

float64 MAX_COM_HEIGHT

# heightOffset specifies CoM height relative to the default starting height, which is
# about 78.9 cm off the ground e.g. heightOffset = -0.1 will put the CoM at about
# 68.9 cm above ground level
float64 heightOffset

# trajectoryTime specifies how fast or how slow to move to the desired pose
float64 trajectoryTime



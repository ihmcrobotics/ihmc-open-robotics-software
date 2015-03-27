## ArmJointTrajectoryPacketMessage
# Packet for executing an arm joint trajectory

# Options for robotSide
# LEFT = 0 - refers to the LEFT side of a robot
# RIGHT = 1 - refers to the RIGHT side of a robot
uint8 robot_side

TrajectoryPointMessage[] trajectory_points



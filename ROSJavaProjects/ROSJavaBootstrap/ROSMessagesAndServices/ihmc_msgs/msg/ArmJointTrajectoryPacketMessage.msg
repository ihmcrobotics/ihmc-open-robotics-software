## ArmJointTrajectoryPacketMessage
# Packet for executing an arm joint trajectory. It works similar to the
# trajectory_msgs/JointTrajectory message.

# Specifies the side of the robot that will execute the trajectory
# Options for robotSide
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot
uint8 robot_side

# List of points in the trajectory
JointTrajectoryPointMessage[] trajectory_points

int64 unique_id



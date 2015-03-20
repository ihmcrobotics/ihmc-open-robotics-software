## ChestOrientationPacketMessage
# This message sets the orientation of the robot's chest in world coordinates.

geometry_msgs/Quaternion orientation

# trajectoryTime specifies how fast or how slow to move to the desired pose
float64 trajectory_time

# toHomePosition can be used to move the chest back to its default starting position
bool to_home_orientation



# Name of the action to adapt
string action_name

# Hand Pose action - Reference frame for the action
string reference_frame_name

# Hand Pose action - The position of the hand specified in reference_frame_name
geometry_msgs/Point new_position

# Hand Pose action - The orientation of the hand specified in reference_frame_name
geometry_msgs/Quaternion new_orientation
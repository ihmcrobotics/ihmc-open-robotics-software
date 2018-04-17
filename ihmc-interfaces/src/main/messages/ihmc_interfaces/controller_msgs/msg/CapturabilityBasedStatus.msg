# This message is part of the IHMC whole-body controller API.
# Published by the IHMC controller, this message carries minimal information relative
# to the current balance status of the robot.
# All the information here is expressed in the world frame.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

geometry_msgs/Point capture_point_2d
geometry_msgs/Point desired_capture_point_2d
geometry_msgs/Point center_of_mass_3d
geometry_msgs/Point[<=8] left_foot_support_polygon_2d
geometry_msgs/Point[<=8] right_foot_support_polygon_2d
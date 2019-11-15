# This message is part of the IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

geometry_msgs/Point left_foot_position_in_world

geometry_msgs/Point right_foot_position_in_world

geometry_msgs/Quaternion left_foot_orientation_in_world

geometry_msgs/Quaternion right_foot_orientation_in_world

geometry_msgs/Point[] left_foot_contact_points_2d

geometry_msgs/Point[] right_foot_contact_points_2d

controller_msgs/FootstepDataListMessage footstep_data_list

controller_msgs/PlanarRegionsListMessage planar_regions_list



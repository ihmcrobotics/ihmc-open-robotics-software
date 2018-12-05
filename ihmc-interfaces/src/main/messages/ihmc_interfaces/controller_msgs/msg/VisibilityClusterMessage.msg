# This message is part of the IHMC footstep planning module.

byte EXTRUSION_INSIDE = 0
byte EXTRUSION_OUTSIDE = 1

byte TYPE_LINE = 0
byte TYPE_MULTI_LINE = 1
byte TYPE_POLYGON = 2

byte extrusion_side 255
byte type 255

geometry_msgs/Pose pose_in_world
geometry_msgs/Point[<=25] raw_points_in_local
geometry_msgs/Point[<=25] navigable_extrusions_in_local
geometry_msgs/Point[<=25] non_navigable_extrusions_in_local
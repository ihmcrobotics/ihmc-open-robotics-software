# This message is part of the IHMC footstep planning module.

int8 EXTRUSION_INSIDE=0

int8 EXTRUSION_OUTSIDE=1

int8 TYPE_LINE=0

int8 TYPE_MULTI_LINE=1

int8 TYPE_POLYGON=2

# Field default value 255
int8 extrusion_side

# Field default value 255
int8 type

geometry_msgs/Pose pose_in_world

controller_msgs/VisibilityClusterPointsMessage raw_points_in_local

controller_msgs/VisibilityClusterPointsMessage[] preferred_navigable_extrusions_in_local

controller_msgs/VisibilityClusterPointsMessage[] preferred_non_navigable_extrusions_in_local

controller_msgs/VisibilityClusterPointsMessage navigable_extrusions_in_local

controller_msgs/VisibilityClusterPointsMessage non_navigable_extrusions_in_local



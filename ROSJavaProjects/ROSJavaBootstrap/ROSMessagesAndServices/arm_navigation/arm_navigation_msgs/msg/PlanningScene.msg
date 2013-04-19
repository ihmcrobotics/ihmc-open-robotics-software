#full robot state
arm_navigation_msgs/RobotState robot_state

#additional frames for duplicating tf
geometry_msgs/TransformStamped[] fixed_frame_transforms

#full allowed collision matrix
AllowedCollisionMatrix allowed_collision_matrix

#allowed contacts
arm_navigation_msgs/AllowedContactSpecification[] allowed_contacts

#all link paddings
arm_navigation_msgs/LinkPadding[] link_padding

#collision objects
arm_navigation_msgs/CollisionObject[] collision_objects
arm_navigation_msgs/AttachedCollisionObject[] attached_collision_objects

#the collision map
arm_navigation_msgs/CollisionMap collision_map

package arm_navigation_msgs;

public interface PlanningScene extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/PlanningScene";
  static final java.lang.String _DEFINITION = "#full robot state\narm_navigation_msgs/RobotState robot_state\n\n#additional frames for duplicating tf\ngeometry_msgs/TransformStamped[] fixed_frame_transforms\n\n#full allowed collision matrix\nAllowedCollisionMatrix allowed_collision_matrix\n\n#allowed contacts\narm_navigation_msgs/AllowedContactSpecification[] allowed_contacts\n\n#all link paddings\narm_navigation_msgs/LinkPadding[] link_padding\n\n#collision objects\narm_navigation_msgs/CollisionObject[] collision_objects\narm_navigation_msgs/AttachedCollisionObject[] attached_collision_objects\n\n#the collision map\narm_navigation_msgs/CollisionMap collision_map\n";
  arm_navigation_msgs.RobotState getRobotState();
  void setRobotState(arm_navigation_msgs.RobotState value);
  java.util.List<geometry_msgs.TransformStamped> getFixedFrameTransforms();
  void setFixedFrameTransforms(java.util.List<geometry_msgs.TransformStamped> value);
  arm_navigation_msgs.AllowedCollisionMatrix getAllowedCollisionMatrix();
  void setAllowedCollisionMatrix(arm_navigation_msgs.AllowedCollisionMatrix value);
  java.util.List<arm_navigation_msgs.AllowedContactSpecification> getAllowedContacts();
  void setAllowedContacts(java.util.List<arm_navigation_msgs.AllowedContactSpecification> value);
  java.util.List<arm_navigation_msgs.LinkPadding> getLinkPadding();
  void setLinkPadding(java.util.List<arm_navigation_msgs.LinkPadding> value);
  java.util.List<arm_navigation_msgs.CollisionObject> getCollisionObjects();
  void setCollisionObjects(java.util.List<arm_navigation_msgs.CollisionObject> value);
  java.util.List<arm_navigation_msgs.AttachedCollisionObject> getAttachedCollisionObjects();
  void setAttachedCollisionObjects(java.util.List<arm_navigation_msgs.AttachedCollisionObject> value);
  arm_navigation_msgs.CollisionMap getCollisionMap();
  void setCollisionMap(arm_navigation_msgs.CollisionMap value);
}

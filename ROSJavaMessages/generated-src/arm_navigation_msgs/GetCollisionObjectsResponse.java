package arm_navigation_msgs;

public interface GetCollisionObjectsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/GetCollisionObjectsResponse";
  static final java.lang.String _DEFINITION = "#points in the collision map if include_points is set to true\narm_navigation_msgs/CollisionMap points\n#vector of collision objects in the collision map\narm_navigation_msgs/CollisionObject[] collision_objects\n#vector of attached collision objects in the collision map\narm_navigation_msgs/AttachedCollisionObject[] attached_collision_objects";
  arm_navigation_msgs.CollisionMap getPoints();
  void setPoints(arm_navigation_msgs.CollisionMap value);
  java.util.List<arm_navigation_msgs.CollisionObject> getCollisionObjects();
  void setCollisionObjects(java.util.List<arm_navigation_msgs.CollisionObject> value);
  java.util.List<arm_navigation_msgs.AttachedCollisionObject> getAttachedCollisionObjects();
  void setAttachedCollisionObjects(java.util.List<arm_navigation_msgs.AttachedCollisionObject> value);
}

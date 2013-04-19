package arm_navigation_msgs;

public interface GetCollisionObjects extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/GetCollisionObjects";
  static final java.lang.String _DEFINITION = "#Whether or not to include the points in the collision map\n#if set to false, collision map in feedback will contain\n#no points\nbool include_points\n---\n#points in the collision map if include_points is set to true\narm_navigation_msgs/CollisionMap points\n#vector of collision objects in the collision map\narm_navigation_msgs/CollisionObject[] collision_objects\n#vector of attached collision objects in the collision map\narm_navigation_msgs/AttachedCollisionObject[] attached_collision_objects\n";
}

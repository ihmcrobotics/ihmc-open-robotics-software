package arm_navigation_msgs;

public interface OrderedCollisionOperations extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/OrderedCollisionOperations";
  static final java.lang.String _DEFINITION = "# A set of collision operations that will be performed in the order they are specified\nCollisionOperation[] collision_operations";
  java.util.List<arm_navigation_msgs.CollisionOperation> getCollisionOperations();
  void setCollisionOperations(java.util.List<arm_navigation_msgs.CollisionOperation> value);
}

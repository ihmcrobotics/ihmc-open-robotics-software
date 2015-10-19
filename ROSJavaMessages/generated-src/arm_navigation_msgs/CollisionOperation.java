package arm_navigation_msgs;

public interface CollisionOperation extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/CollisionOperation";
  static final java.lang.String _DEFINITION = "# A definition of a collision operation\n# E.g. (\"gripper\",COLLISION_SET_ALL,ENABLE) will enable collisions \n# between the gripper and all objects in the collision space\n\nstring object1\nstring object2\nstring COLLISION_SET_ALL=\"all\"\nstring COLLISION_SET_OBJECTS=\"objects\"\nstring COLLISION_SET_ATTACHED_OBJECTS=\"attached\"\n\n# The penetration distance to which collisions are allowed. This is 0.0 by default.\nfloat64 penetration_distance\n\n# Flag that determines whether collisions will be enabled or disabled for the pair of objects specified above\nint32 operation\nint32 DISABLE=0\nint32 ENABLE=1\n";
  static final java.lang.String COLLISION_SET_ALL = "\"all\"";
  static final java.lang.String COLLISION_SET_OBJECTS = "\"objects\"";
  static final java.lang.String COLLISION_SET_ATTACHED_OBJECTS = "\"attached\"";
  static final int DISABLE = 0;
  static final int ENABLE = 1;
  java.lang.String getObject1();
  void setObject1(java.lang.String value);
  java.lang.String getObject2();
  void setObject2(java.lang.String value);
  double getPenetrationDistance();
  void setPenetrationDistance(double value);
  int getOperation();
  void setOperation(int value);
}

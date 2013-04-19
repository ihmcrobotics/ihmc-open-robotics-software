package object_manipulation_msgs;

public interface GripperTranslation extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/GripperTranslation";
  static final java.lang.String _DEFINITION = "# defines a translation for the gripper, used in pickup or place tasks\n# for example for lifting an object off a table or approaching the table for placing\n\n# the direction of the translation\ngeometry_msgs/Vector3Stamped direction\n\n# the desired translation distance\nfloat32 desired_distance\n\n# the min distance that must be considered feasible before the\n# grasp is even attempted\nfloat32 min_distance";
  geometry_msgs.Vector3Stamped getDirection();
  void setDirection(geometry_msgs.Vector3Stamped value);
  float getDesiredDistance();
  void setDesiredDistance(float value);
  float getMinDistance();
  void setMinDistance(float value);
}

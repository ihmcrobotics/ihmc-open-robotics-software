package geometry_msgs;

public interface Transform extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/Transform";
  static final java.lang.String _DEFINITION = "# This represents the transform between two coordinate frames in free space.\n\nVector3 translation\nQuaternion rotation\n";
  geometry_msgs.Vector3 getTranslation();
  void setTranslation(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getRotation();
  void setRotation(geometry_msgs.Quaternion value);
}

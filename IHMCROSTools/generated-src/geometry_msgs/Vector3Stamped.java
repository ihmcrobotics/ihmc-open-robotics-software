package geometry_msgs;

public interface Vector3Stamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/Vector3Stamped";
  static final java.lang.String _DEFINITION = "# This represents a Vector3 with reference coordinate frame and timestamp\nHeader header\nVector3 vector\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Vector3 getVector();
  void setVector(geometry_msgs.Vector3 value);
}

package geometry_msgs;

public interface Vector3 extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/Vector3";
  static final java.lang.String _DEFINITION = "# This represents a vector in free space. \n\nfloat64 x\nfloat64 y\nfloat64 z";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getZ();
  void setZ(double value);
}

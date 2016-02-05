package geometry_msgs;

public interface Quaternion extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/Quaternion";
  static final java.lang.String _DEFINITION = "# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getZ();
  void setZ(double value);
  double getW();
  void setW(double value);
}

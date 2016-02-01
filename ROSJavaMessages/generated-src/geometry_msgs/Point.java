package geometry_msgs;

public interface Point extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/Point";
  static final java.lang.String _DEFINITION = "# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getZ();
  void setZ(double value);
}

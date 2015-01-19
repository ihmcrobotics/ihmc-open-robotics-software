package map_msgs;

public interface GetMapROIRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_msgs/GetMapROIRequest";
  static final java.lang.String _DEFINITION = "float64 x\nfloat64 y\nfloat64 l_x\nfloat64 l_y\n";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getLX();
  void setLX(double value);
  double getLY();
  void setLY(double value);
}

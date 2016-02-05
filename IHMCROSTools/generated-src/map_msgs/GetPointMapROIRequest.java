package map_msgs;

public interface GetPointMapROIRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_msgs/GetPointMapROIRequest";
  static final java.lang.String _DEFINITION = "float64 x\nfloat64 y\nfloat64 z\nfloat64 r    # if != 0, circular ROI of radius r\nfloat64 l_x  # if r == 0, length of AABB on x\nfloat64 l_y  # if r == 0, length of AABB on y\nfloat64 l_z  # if r == 0, length of AABB on z\n";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getZ();
  void setZ(double value);
  double getR();
  void setR(double value);
  double getLX();
  void setLX(double value);
  double getLY();
  void setLY(double value);
  double getLZ();
  void setLZ(double value);
}

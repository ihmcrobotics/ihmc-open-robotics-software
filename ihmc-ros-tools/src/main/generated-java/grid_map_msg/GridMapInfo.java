package grid_map_msg;

public interface GridMapInfo extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "grid_map_msg/GridMapInfo";
  static final java.lang.String _DEFINITION = "# Header (time and frame)\nHeader header\n\n# Resolution of the grid [m/cell].\nfloat64 resolution\n\n# Length in x-direction [m].\nfloat64 lengthX\n\n# Width in y-direction [m].\nfloat64 lengthY\n\n# Position of the grid map in x-direction in the parent frame [m].\nfloat64 positionX\n\n# Position of the grid map in y-direction in the parent frame [m].\nfloat64 positionY";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double getResolution();
  void setResolution(double value);
  double getLengthX();
  void setLengthX(double value);
  double getLengthY();
  void setLengthY(double value);
  double getPositionX();
  void setPositionX(double value);
  double getPositionY();
  void setPositionY(double value);
}

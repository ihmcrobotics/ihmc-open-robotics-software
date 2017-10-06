package grid_map_msg;

public interface GetGridMapRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "grid_map_msg/GetGridMapRequest";
  static final java.lang.String _DEFINITION = "# Requested submap position in x-direction [m].\nfloat64 positionX\n  \n# Requested submap position in y-direction [m].\nfloat64 positionY\n\n# Requested submap length in x-direction [m].\nfloat64 lengthX\n  \n# Requested submap width in y-direction [m].\nfloat64 lengthY\n\n# Requested data. If empty, get all available data.\nstd_msgs/String[] dataDefinition\n\n";
  double getPositionX();
  void setPositionX(double value);
  double getPositionY();
  void setPositionY(double value);
  double getLengthX();
  void setLengthX(double value);
  double getLengthY();
  void setLengthY(double value);
  java.util.List<std_msgs.String> getDataDefinition();
  void setDataDefinition(java.util.List<std_msgs.String> value);
}

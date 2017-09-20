package handle_msgs;

public interface HandleCollisions extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "handle_msgs/HandleCollisions";
  static final java.lang.String _DEFINITION = "# This is sensors of the HANDLE hand after calibration and data manipulation\n# published from the package sensors, by the sensors_publisher\n\n# not all the sensors were included, but only the one which were addressed at the moment\n\n# Currently only used for time stamp.  \nHeader header\n\nCollision[] collisions\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<handle_msgs.Collision> getCollisions();
  void setCollisions(java.util.List<handle_msgs.Collision> value);
}

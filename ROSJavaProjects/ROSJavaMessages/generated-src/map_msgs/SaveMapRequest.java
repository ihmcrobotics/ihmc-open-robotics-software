package map_msgs;

public interface SaveMapRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_msgs/SaveMapRequest";
  static final java.lang.String _DEFINITION = "# Save the map to the filesystem\nstd_msgs/String filename ";
  std_msgs.String getFilename();
  void setFilename(std_msgs.String value);
}

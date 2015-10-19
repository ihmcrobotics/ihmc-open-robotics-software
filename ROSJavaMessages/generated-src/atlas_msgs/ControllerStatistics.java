package atlas_msgs;

public interface ControllerStatistics extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/ControllerStatistics";
  static final java.lang.String _DEFINITION = "# Keep track of Atlas controller statistics over ros topic\nHeader header\nfloat64 command_age\nfloat64 command_age_mean\nfloat64 command_age_variance\nfloat64 command_age_window_size\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double getCommandAge();
  void setCommandAge(double value);
  double getCommandAgeMean();
  void setCommandAgeMean(double value);
  double getCommandAgeVariance();
  void setCommandAgeVariance(double value);
  double getCommandAgeWindowSize();
  void setCommandAgeWindowSize(double value);
}

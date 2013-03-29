package pr2_msgs;

public interface LaserTrajCmd extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/LaserTrajCmd";
  static final java.lang.String _DEFINITION = "# This message is used to set the profile that the tilt laser controller\n# executes.\nHeader header\nstring profile              # options are currently \"linear\" or \"linear_blended\"\nfloat64[] position          # Laser positions\nduration[] time_from_start  # Times to reach laser positions in seconds\nfloat64 max_velocity        # Set <= 0 to use default\nfloat64 max_acceleration    # Set <= 0 to use default\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getProfile();
  void setProfile(java.lang.String value);
  double[] getPosition();
  void setPosition(double[] value);
  java.util.List<org.ros.message.Duration> getTimeFromStart();
  void setTimeFromStart(java.util.List<org.ros.message.Duration> value);
  double getMaxVelocity();
  void setMaxVelocity(double value);
  double getMaxAcceleration();
  void setMaxAcceleration(double value);
}

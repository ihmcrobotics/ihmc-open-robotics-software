package pr2_msgs;

public interface PeriodicCmd extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/PeriodicCmd";
  static final java.lang.String _DEFINITION = "# This message is used to set the parameters of a profile executed by the\n# laser tilt controller.\nHeader header\nstring profile\nfloat64 period\nfloat64 amplitude\nfloat64 offset\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getProfile();
  void setProfile(java.lang.String value);
  double getPeriod();
  void setPeriod(double value);
  double getAmplitude();
  void setAmplitude(double value);
  double getOffset();
  void setOffset(double value);
}

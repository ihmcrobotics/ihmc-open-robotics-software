package pr2_msgs;

public interface PowerState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/PowerState";
  static final java.lang.String _DEFINITION = "# This message communicates the state of the PR2\'s power system.\nHeader header\nfloat64 power_consumption ## Watts\nduration time_remaining ## estimated time to empty or full\nstring prediction_method ## how time_remaining is being calculated\nint8  relative_capacity ## percent of capacity\nint8  AC_present ## number of packs detecting AC power, > 0 means plugged in\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double getPowerConsumption();
  void setPowerConsumption(double value);
  org.ros.message.Duration getTimeRemaining();
  void setTimeRemaining(org.ros.message.Duration value);
  java.lang.String getPredictionMethod();
  void setPredictionMethod(java.lang.String value);
  byte getRelativeCapacity();
  void setRelativeCapacity(byte value);
  byte getACPresent();
  void setACPresent(byte value);
}

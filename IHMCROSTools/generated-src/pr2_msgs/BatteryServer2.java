package pr2_msgs;

public interface BatteryServer2 extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/BatteryServer2";
  static final java.lang.String _DEFINITION = "# This message communicates the state of a battery server, which controls\n# multiple batteries.\nHeader header\nint32 MAX_BAT_COUNT=4\nint32 MAX_BAT_REG=48\nint32 id  # unique ID for each battery server\n# Battery System Stats\ntime last_system_update  # last time the system stats where updated\nduration time_left       # in seconds (hardware resolution is 1 minute)\nint32 average_charge     # in percent\nstring message           # message from the ocean server\ntime last_controller_update # last time a battery status flag was updated\n# for each battery\npr2_msgs/BatteryState2[] battery\n";
  static final int MAX_BAT_COUNT = 4;
  static final int MAX_BAT_REG = 48;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getId();
  void setId(int value);
  org.ros.message.Time getLastSystemUpdate();
  void setLastSystemUpdate(org.ros.message.Time value);
  org.ros.message.Duration getTimeLeft();
  void setTimeLeft(org.ros.message.Duration value);
  int getAverageCharge();
  void setAverageCharge(int value);
  java.lang.String getMessage();
  void setMessage(java.lang.String value);
  org.ros.message.Time getLastControllerUpdate();
  void setLastControllerUpdate(org.ros.message.Time value);
  java.util.List<pr2_msgs.BatteryState2> getBattery();
  void setBattery(java.util.List<pr2_msgs.BatteryState2> value);
}

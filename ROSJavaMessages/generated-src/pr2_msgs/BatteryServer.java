package pr2_msgs;

public interface BatteryServer extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/BatteryServer";
  static final java.lang.String _DEFINITION = "# DEPRECATED. Use pr2_msgs/BatteryServer2 instead.\nHeader header\nuint32 MAX_BAT_COUNT=4\nuint32 MAX_BAT_REG=48\nint32 id  # unique ID for each battery server\n# Battery System Stats\nint32 lastTimeSystem #epoch time\nuint16 timeLeft      # in minutes\nuint16 averageCharge # in percent\nstring message\n# Battery Controller Flags\nint32 lastTimeController #epoch time\nuint16 present\nuint16 charging\nuint16 discharging\nuint16 reserved\nuint16 powerPresent\nuint16 powerNG\nuint16 inhibited\n# for each battery\npr2_msgs/BatteryState[] battery\n";
  static final int MAX_BAT_COUNT = 4;
  static final int MAX_BAT_REG = 48;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getId();
  void setId(int value);
  int getLastTimeSystem();
  void setLastTimeSystem(int value);
  short getTimeLeft();
  void setTimeLeft(short value);
  short getAverageCharge();
  void setAverageCharge(short value);
  java.lang.String getMessage();
  void setMessage(java.lang.String value);
  int getLastTimeController();
  void setLastTimeController(int value);
  short getPresent();
  void setPresent(short value);
  short getCharging();
  void setCharging(short value);
  short getDischarging();
  void setDischarging(short value);
  short getReserved();
  void setReserved(short value);
  short getPowerPresent();
  void setPowerPresent(short value);
  short getPowerNG();
  void setPowerNG(short value);
  short getInhibited();
  void setInhibited(short value);
  java.util.List<pr2_msgs.BatteryState> getBattery();
  void setBattery(java.util.List<pr2_msgs.BatteryState> value);
}

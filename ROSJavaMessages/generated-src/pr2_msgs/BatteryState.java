package pr2_msgs;

public interface BatteryState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/BatteryState";
  static final java.lang.String _DEFINITION = "# DEPRECATED.  Use pr2_msgs/BatteryState2 instead.\n# Each batteries registers\nint32 lastTimeBattery #epoch time\nuint16[48] batReg\nuint16[48] batRegFlag\nint32[48] batRegTime\n";
  int getLastTimeBattery();
  void setLastTimeBattery(int value);
  short[] getBatReg();
  void setBatReg(short[] value);
  short[] getBatRegFlag();
  void setBatRegFlag(short[] value);
  int[] getBatRegTime();
  void setBatRegTime(int[] value);
}

package pr2_msgs;

public interface PowerBoardState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/PowerBoardState";
  static final java.lang.String _DEFINITION = "# This message communicates the state of the PR2\'s power board.\nint8 STATE_NOPOWER=0\nint8 STATE_STANDBY=1\nint8 STATE_PUMPING=2\nint8 STATE_ON=3\nint8 STATE_ENABLED=3  # Preferred over STATE_ON, keeping STATE_ON for backcompat\nint8 STATE_DISABLED=4\n\nint8 MASTER_NOPOWER=0\nint8 MASTER_STANDBY=1\nint8 MASTER_ON=2\nint8 MASTER_OFF=3\nint8 MASTER_SHUTDOWN=4\n\nHeader header\nstring name # Name with serial number\nuint32 serial_num # Serial number for this board\'s message\nfloat64 input_voltage # Input voltage to power board\n\n# Master States:\n#  MASTER_NOPOWER, MASTER_STANDBY, MASTER_ON, MASTER_OFF, MASTER_SHUTDOWN \nint8 master_state  # The master state machine\'s state in the powerboard\n\n# Circuit States:\n#  STATE_NOPOWER, STATE_STANDBY, STATE_PUMPING, STATE_ON, STATE_DISABLED\nint8[3] circuit_state # One of the above states\nfloat64[3] circuit_voltage  # Output voltage of each circuit\n\n# True if robot should be enabled\nbool run_stop           #Note - if the wireless run-stop is hit, this will be unobservable\nbool wireless_stop \n";
  static final byte STATE_NOPOWER = 0;
  static final byte STATE_STANDBY = 1;
  static final byte STATE_PUMPING = 2;
  static final byte STATE_ON = 3;
  static final byte STATE_ENABLED = 3;
  static final byte STATE_DISABLED = 4;
  static final byte MASTER_NOPOWER = 0;
  static final byte MASTER_STANDBY = 1;
  static final byte MASTER_ON = 2;
  static final byte MASTER_OFF = 3;
  static final byte MASTER_SHUTDOWN = 4;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getName();
  void setName(java.lang.String value);
  int getSerialNum();
  void setSerialNum(int value);
  double getInputVoltage();
  void setInputVoltage(double value);
  byte getMasterState();
  void setMasterState(byte value);
  org.jboss.netty.buffer.ChannelBuffer getCircuitState();
  void setCircuitState(org.jboss.netty.buffer.ChannelBuffer value);
  double[] getCircuitVoltage();
  void setCircuitVoltage(double[] value);
  boolean getRunStop();
  void setRunStop(boolean value);
  boolean getWirelessStop();
  void setWirelessStop(boolean value);
}

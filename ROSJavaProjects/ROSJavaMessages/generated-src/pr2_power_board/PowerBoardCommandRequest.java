package pr2_power_board;

public interface PowerBoardCommandRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_power_board/PowerBoardCommandRequest";
  static final java.lang.String _DEFINITION = "uint32 serial_number #serial number of the board to control\nint32 breaker_number ## 0=Base, 1=RightArm 2=LeftArm\nstring command  # Options start, stop, reset, disable, none\nuint32 flags\n";
  int getSerialNumber();
  void setSerialNumber(int value);
  int getBreakerNumber();
  void setBreakerNumber(int value);
  java.lang.String getCommand();
  void setCommand(java.lang.String value);
  int getFlags();
  void setFlags(int value);
}

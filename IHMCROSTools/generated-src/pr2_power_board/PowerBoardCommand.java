package pr2_power_board;

public interface PowerBoardCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_power_board/PowerBoardCommand";
  static final java.lang.String _DEFINITION = "uint32 serial_number #serial number of the board to control\nint32 breaker_number ## 0=Base, 1=RightArm 2=LeftArm\nstring command  # Options start, stop, reset, disable, none\nuint32 flags\n---\nint32 retval # 0 = false, 1=true\n";
}

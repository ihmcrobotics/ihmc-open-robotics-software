package pr2_power_board;

public interface PowerBoardCommandResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_power_board/PowerBoardCommandResponse";
  static final java.lang.String _DEFINITION = "int32 retval # 0 = false, 1=true";
  int getRetval();
  void setRetval(int value);
}

package pr2_power_board;

public interface PowerBoardCommand2Response extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_power_board/PowerBoardCommand2Response";
  static final java.lang.String _DEFINITION = "# return if the command was successfully sent\nbool success";
  boolean getSuccess();
  void setSuccess(boolean value);
}

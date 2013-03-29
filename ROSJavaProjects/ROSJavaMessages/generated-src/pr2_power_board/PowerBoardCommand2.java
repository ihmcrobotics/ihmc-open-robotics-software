package pr2_power_board;

public interface PowerBoardCommand2 extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_power_board/PowerBoardCommand2";
  static final java.lang.String _DEFINITION = "int32 NUMBER_OF_CIRCUITS = 3\n#\n# Select which circuit to command\n#\nint32 circuit\n#\n# Defined circuits\n#\nint32 BASE      = 0\nint32 RIGHT_ARM = 1\nint32 LEFT_ARM  = 2\n#\n# Command to send to circuit:\n# command = start, stop, reset, disable, none\n#\nstring command  \n#\n# reset the latched voltage and current statistics\nbool reset_stats\n#\n# reset the latched stats for each circuit\nbool reset_circuits\n---\n# return if the command was successfully sent\nbool success\n";
}

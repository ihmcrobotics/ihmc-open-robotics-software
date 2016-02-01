package pr2_power_board;

public interface PowerBoardCommand2Request extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_power_board/PowerBoardCommand2Request";
  static final java.lang.String _DEFINITION = "int32 NUMBER_OF_CIRCUITS = 3\n#\n# Select which circuit to command\n#\nint32 circuit\n#\n# Defined circuits\n#\nint32 BASE      = 0\nint32 RIGHT_ARM = 1\nint32 LEFT_ARM  = 2\n#\n# Command to send to circuit:\n# command = start, stop, reset, disable, none\n#\nstring command  \n#\n# reset the latched voltage and current statistics\nbool reset_stats\n#\n# reset the latched stats for each circuit\nbool reset_circuits\n";
  static final int NUMBER_OF_CIRCUITS = 3;
  static final int BASE = 0;
  static final int RIGHT_ARM = 1;
  static final int LEFT_ARM = 2;
  int getCircuit();
  void setCircuit(int value);
  java.lang.String getCommand();
  void setCommand(java.lang.String value);
  boolean getResetStats();
  void setResetStats(boolean value);
  boolean getResetCircuits();
  void setResetCircuits(boolean value);
}

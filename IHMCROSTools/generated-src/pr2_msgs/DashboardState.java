package pr2_msgs;

public interface DashboardState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/DashboardState";
  static final java.lang.String _DEFINITION = "# This message communicates state information that might be used by a\n# dashboard application.\nstd_msgs/Bool motors_halted\nbool motors_halted_valid\n\npr2_msgs/PowerBoardState power_board_state\nbool power_board_state_valid\n\npr2_msgs/PowerState power_state\nbool power_state_valid\n\npr2_msgs/AccessPoint access_point\nbool access_point_valid\n";
  std_msgs.Bool getMotorsHalted();
  void setMotorsHalted(std_msgs.Bool value);
  boolean getMotorsHaltedValid();
  void setMotorsHaltedValid(boolean value);
  pr2_msgs.PowerBoardState getPowerBoardState();
  void setPowerBoardState(pr2_msgs.PowerBoardState value);
  boolean getPowerBoardStateValid();
  void setPowerBoardStateValid(boolean value);
  pr2_msgs.PowerState getPowerState();
  void setPowerState(pr2_msgs.PowerState value);
  boolean getPowerStateValid();
  void setPowerStateValid(boolean value);
  pr2_msgs.AccessPoint getAccessPoint();
  void setAccessPoint(pr2_msgs.AccessPoint value);
  boolean getAccessPointValid();
  void setAccessPointValid(boolean value);
}

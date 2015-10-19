package atlas_msgs;

public interface ResetControlsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/ResetControlsRequest";
  static final java.lang.String _DEFINITION = "# permissible values for mode\n\nbool reset_bdi_controller             # reset BDI controller internal states.\nbool reset_pid_controller             # reset PID controller internal states.\nbool reload_pid_from_ros              # reload PID gains from ROS param server.\natlas_msgs/AtlasCommand atlas_command # unless reload_pid_from_ros is true,\n                                      # values from atlas_command will be read\n                                      # into active PID controller.\n\n";
  boolean getResetBdiController();
  void setResetBdiController(boolean value);
  boolean getResetPidController();
  void setResetPidController(boolean value);
  boolean getReloadPidFromRos();
  void setReloadPidFromRos(boolean value);
  atlas_msgs.AtlasCommand getAtlasCommand();
  void setAtlasCommand(atlas_msgs.AtlasCommand value);
}

package atlas_msgs;

public interface ResetControlsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/ResetControlsRequest";
  static final java.lang.String _DEFINITION = "osrf_msgs/JointCommands joint_commands\n";
  osrf_msgs.JointCommands getJointCommands();
  void setJointCommands(osrf_msgs.JointCommands value);
}

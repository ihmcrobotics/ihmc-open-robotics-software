package pr2_mechanism_msgs;

public interface MechanismStatistics extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_msgs/MechanismStatistics";
  static final java.lang.String _DEFINITION = "# This message describes the state of the pr2 mechanism. It contains the state of\n# each actuator, each joint, and each controller that is spawned in pr2_mechanism_control.\n\nHeader header\nActuatorStatistics[] actuator_statistics\nJointStatistics[] joint_statistics\nControllerStatistics[] controller_statistics\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<pr2_mechanism_msgs.ActuatorStatistics> getActuatorStatistics();
  void setActuatorStatistics(java.util.List<pr2_mechanism_msgs.ActuatorStatistics> value);
  java.util.List<pr2_mechanism_msgs.JointStatistics> getJointStatistics();
  void setJointStatistics(java.util.List<pr2_mechanism_msgs.JointStatistics> value);
  java.util.List<pr2_mechanism_msgs.ControllerStatistics> getControllerStatistics();
  void setControllerStatistics(java.util.List<pr2_mechanism_msgs.ControllerStatistics> value);
}

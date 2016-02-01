package arm_navigation_msgs;

public interface RobotState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/RobotState";
  static final java.lang.String _DEFINITION = "# This message contains information about the robot state, i.e. the positions of its joints and links\nsensor_msgs/JointState joint_state\narm_navigation_msgs/MultiDOFJointState multi_dof_joint_state\n";
  sensor_msgs.JointState getJointState();
  void setJointState(sensor_msgs.JointState value);
  arm_navigation_msgs.MultiDOFJointState getMultiDofJointState();
  void setMultiDofJointState(arm_navigation_msgs.MultiDOFJointState value);
}

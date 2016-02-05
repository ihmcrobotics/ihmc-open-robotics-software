package arm_navigation_msgs;

public interface RobotTrajectory extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/RobotTrajectory";
  static final java.lang.String _DEFINITION = "trajectory_msgs/JointTrajectory joint_trajectory\narm_navigation_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory\n";
  trajectory_msgs.JointTrajectory getJointTrajectory();
  void setJointTrajectory(trajectory_msgs.JointTrajectory value);
  arm_navigation_msgs.MultiDOFJointTrajectory getMultiDofJointTrajectory();
  void setMultiDofJointTrajectory(arm_navigation_msgs.MultiDOFJointTrajectory value);
}

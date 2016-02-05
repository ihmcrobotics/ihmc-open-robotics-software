package arm_navigation_msgs;

public interface FilterJointTrajectoryRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/FilterJointTrajectoryRequest";
  static final java.lang.String _DEFINITION = "# The trajectory to filter\ntrajectory_msgs/JointTrajectory trajectory\n\n# Starting state updates. If certain joints should be considered\n# at positions other than the current ones, these positions should\n# be set here\narm_navigation_msgs/RobotState start_state\n\n# A vector of JointLimit messages.\n\n# Each message contains the limits for a specific joint\narm_navigation_msgs/JointLimits[] limits\n\n# Total allowed time for filtering\nduration allowed_time\n";
  trajectory_msgs.JointTrajectory getTrajectory();
  void setTrajectory(trajectory_msgs.JointTrajectory value);
  arm_navigation_msgs.RobotState getStartState();
  void setStartState(arm_navigation_msgs.RobotState value);
  java.util.List<arm_navigation_msgs.JointLimits> getLimits();
  void setLimits(java.util.List<arm_navigation_msgs.JointLimits> value);
  org.ros.message.Duration getAllowedTime();
  void setAllowedTime(org.ros.message.Duration value);
}

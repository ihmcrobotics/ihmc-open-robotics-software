package arm_navigation_msgs;

public interface FilterJointTrajectoryWithConstraintsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/FilterJointTrajectoryWithConstraintsRequest";
  static final java.lang.String _DEFINITION = "# A trajectory message that encodes joint limits, collision and state constraints within it.\ntrajectory_msgs/JointTrajectory trajectory\n\n# Group name describing trajectory\nstring group_name\n\n# A vector of JointLimit messages.\n# Each message contains the limits for a specific joint\narm_navigation_msgs/JointLimits[] limits\n\n# Starting state updates. If certain joints should be considered\n# at positions other than the current ones, these positions should\n# be set here\narm_navigation_msgs/RobotState start_state\n\n# A set of path constraints on the trajectory\nConstraints path_constraints\n\n# A set of goal constraints on the trajectory\nConstraints goal_constraints\n\nduration allowed_time\n";
  trajectory_msgs.JointTrajectory getTrajectory();
  void setTrajectory(trajectory_msgs.JointTrajectory value);
  java.lang.String getGroupName();
  void setGroupName(java.lang.String value);
  java.util.List<arm_navigation_msgs.JointLimits> getLimits();
  void setLimits(java.util.List<arm_navigation_msgs.JointLimits> value);
  arm_navigation_msgs.RobotState getStartState();
  void setStartState(arm_navigation_msgs.RobotState value);
  arm_navigation_msgs.Constraints getPathConstraints();
  void setPathConstraints(arm_navigation_msgs.Constraints value);
  arm_navigation_msgs.Constraints getGoalConstraints();
  void setGoalConstraints(arm_navigation_msgs.Constraints value);
  org.ros.message.Duration getAllowedTime();
  void setAllowedTime(org.ros.message.Duration value);
}

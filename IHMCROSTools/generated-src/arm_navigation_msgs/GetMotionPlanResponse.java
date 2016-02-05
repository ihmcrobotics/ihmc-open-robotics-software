package arm_navigation_msgs;

public interface GetMotionPlanResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/GetMotionPlanResponse";
  static final java.lang.String _DEFINITION = "\n# A solution trajectory, if found\narm_navigation_msgs/RobotTrajectory trajectory\n\n# The corresponding robot state\narm_navigation_msgs/RobotState robot_state\n\n# Planning time\nduration planning_time\n\n# Error code - encodes the overall reason for failure\narm_navigation_msgs/ArmNavigationErrorCodes error_code\n\n# More detailed error codes (optional) - encode information about each point in the returned trajectory\narm_navigation_msgs/ArmNavigationErrorCodes[] trajectory_error_codes";
  arm_navigation_msgs.RobotTrajectory getTrajectory();
  void setTrajectory(arm_navigation_msgs.RobotTrajectory value);
  arm_navigation_msgs.RobotState getRobotState();
  void setRobotState(arm_navigation_msgs.RobotState value);
  org.ros.message.Duration getPlanningTime();
  void setPlanningTime(org.ros.message.Duration value);
  arm_navigation_msgs.ArmNavigationErrorCodes getErrorCode();
  void setErrorCode(arm_navigation_msgs.ArmNavigationErrorCodes value);
  java.util.List<arm_navigation_msgs.ArmNavigationErrorCodes> getTrajectoryErrorCodes();
  void setTrajectoryErrorCodes(java.util.List<arm_navigation_msgs.ArmNavigationErrorCodes> value);
}

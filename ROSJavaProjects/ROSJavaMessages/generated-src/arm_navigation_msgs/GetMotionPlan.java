package arm_navigation_msgs;

public interface GetMotionPlan extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/GetMotionPlan";
  static final java.lang.String _DEFINITION = "# This service contains the definition for a request to the motion\n# planner and the output it provides\n\nMotionPlanRequest motion_plan_request\n\n---\n\n# A solution trajectory, if found\narm_navigation_msgs/RobotTrajectory trajectory\n\n# The corresponding robot state\narm_navigation_msgs/RobotState robot_state\n\n# Planning time\nduration planning_time\n\n# Error code - encodes the overall reason for failure\narm_navigation_msgs/ArmNavigationErrorCodes error_code\n\n# More detailed error codes (optional) - encode information about each point in the returned trajectory\narm_navigation_msgs/ArmNavigationErrorCodes[] trajectory_error_codes\n";
}

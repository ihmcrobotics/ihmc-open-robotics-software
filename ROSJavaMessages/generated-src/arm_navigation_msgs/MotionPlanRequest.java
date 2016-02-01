package arm_navigation_msgs;

public interface MotionPlanRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/MotionPlanRequest";
  static final java.lang.String _DEFINITION = "# This service contains the definition for a request to the motion\n# planner and the output it provides\n\n# Parameters for the workspace that the planner should work inside\narm_navigation_msgs/WorkspaceParameters workspace_parameters\n\n# Starting state updates. If certain joints should be considered\n# at positions other than the current ones, these positions should\n# be set here\narm_navigation_msgs/RobotState start_state\n\n# The goal state for the model to plan for. The goal is achieved\n# if all constraints are satisfied\narm_navigation_msgs/Constraints goal_constraints\n\n# No state at any point along the path in the produced motion plan will violate these constraints\narm_navigation_msgs/Constraints path_constraints\n\n# The name of the motion planner to use. If no name is specified,\n# a default motion planner will be used\nstring planner_id\n\n# The name of the group of joints on which this planner is operating\nstring group_name\n\n# The number of times this plan is to be computed. Shortest solution\n# will be reported.\nint32 num_planning_attempts\n\n# The maximum amount of time the motion planner is allowed to plan for\nduration allowed_planning_time\n\n# An expected path duration (in seconds) along with an expected discretization of the path allows the planner to determine the discretization of the trajectory that it returns\nduration expected_path_duration\nduration expected_path_dt\n";
  arm_navigation_msgs.WorkspaceParameters getWorkspaceParameters();
  void setWorkspaceParameters(arm_navigation_msgs.WorkspaceParameters value);
  arm_navigation_msgs.RobotState getStartState();
  void setStartState(arm_navigation_msgs.RobotState value);
  arm_navigation_msgs.Constraints getGoalConstraints();
  void setGoalConstraints(arm_navigation_msgs.Constraints value);
  arm_navigation_msgs.Constraints getPathConstraints();
  void setPathConstraints(arm_navigation_msgs.Constraints value);
  java.lang.String getPlannerId();
  void setPlannerId(java.lang.String value);
  java.lang.String getGroupName();
  void setGroupName(java.lang.String value);
  int getNumPlanningAttempts();
  void setNumPlanningAttempts(int value);
  org.ros.message.Duration getAllowedPlanningTime();
  void setAllowedPlanningTime(org.ros.message.Duration value);
  org.ros.message.Duration getExpectedPathDuration();
  void setExpectedPathDuration(org.ros.message.Duration value);
  org.ros.message.Duration getExpectedPathDt();
  void setExpectedPathDt(org.ros.message.Duration value);
}

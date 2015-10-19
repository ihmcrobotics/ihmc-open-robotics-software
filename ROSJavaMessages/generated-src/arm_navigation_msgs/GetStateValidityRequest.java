package arm_navigation_msgs;

public interface GetStateValidityRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/GetStateValidityRequest";
  static final java.lang.String _DEFINITION = "# Checks whether the specified state is a valid state\n# The input state message\narm_navigation_msgs/RobotState robot_state\n\n# Collision checks will be performed if this flag is true\nbool check_collisions\n\n# Path constraints will be checked if this flag is true\nbool check_path_constraints\n\n# Goal constraints will be checked if this flag is true\nbool check_goal_constraints\n\n# Joint limits will be checked if this flag is true\nbool check_joint_limits\n\n# OPTIONAL group name used to disable collisions for other links\nstring group_name\n\n# OPTIONAL specification of a set of path constraints imposed on the robot, \n# e.g. joint or pose constraints. These constraints will be tested only if\n# check_path_constraints = true\narm_navigation_msgs/Constraints path_constraints\n\n# OPTIONAL specification of a set of goal constraints imposed on the robot, \n# e.g. joint or pose constraints. These constraints will be tested only if \n# check_goal_constraints = true\narm_navigation_msgs/Constraints goal_constraints\n\n";
  arm_navigation_msgs.RobotState getRobotState();
  void setRobotState(arm_navigation_msgs.RobotState value);
  boolean getCheckCollisions();
  void setCheckCollisions(boolean value);
  boolean getCheckPathConstraints();
  void setCheckPathConstraints(boolean value);
  boolean getCheckGoalConstraints();
  void setCheckGoalConstraints(boolean value);
  boolean getCheckJointLimits();
  void setCheckJointLimits(boolean value);
  java.lang.String getGroupName();
  void setGroupName(java.lang.String value);
  arm_navigation_msgs.Constraints getPathConstraints();
  void setPathConstraints(arm_navigation_msgs.Constraints value);
  arm_navigation_msgs.Constraints getGoalConstraints();
  void setGoalConstraints(arm_navigation_msgs.Constraints value);
}

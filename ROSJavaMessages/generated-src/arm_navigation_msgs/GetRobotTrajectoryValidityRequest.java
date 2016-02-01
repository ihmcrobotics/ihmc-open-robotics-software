package arm_navigation_msgs;

public interface GetRobotTrajectoryValidityRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/GetRobotTrajectoryValidityRequest";
  static final java.lang.String _DEFINITION = "# The trajectory for which validity is to be checked\narm_navigation_msgs/RobotTrajectory trajectory\n\n# The state of the robot. \n# This state message contains information on the position of the joints of the robot. \n# Any joint information in the path message above will overwrite corresponding information \n# for the same joint in the state message.\n# Any joint not contained in either the path or state message will be assumed to be at \n# the current position of the joint. \narm_navigation_msgs/RobotState robot_state\n\n# Collision checks will be performed if this flag is true\nbool check_collisions\n\n# Path constraints will be checked if this flag is true\nbool check_path_constraints\n\n# Goal constraints will be checked if this flag is true\nbool check_goal_constraints\n\n# Joint limits will be checked if this flag is true\nbool check_joint_limits\n\n# If this flag is true, the entire trajectory will be checked before the node returns\n# The default value of this flag is false and so the node will return as soon \n# as the first check on the points in the trajectory fails\nbool check_full_trajectory\n\n# OPTIONAL specification of a set of path constraints imposed on the robot, \n# e.g. joint or pose constraints. These constraints will be tested only if\n# check_path_constraints = true\narm_navigation_msgs/Constraints path_constraints\n\n# OPTIONAL specification of a set of goal constraints imposed on the robot, \n# e.g. joint or pose constraints. These constraints will be tested only if \n# check_goal_constraints = true\narm_navigation_msgs/Constraints goal_constraints\n\n";
  arm_navigation_msgs.RobotTrajectory getTrajectory();
  void setTrajectory(arm_navigation_msgs.RobotTrajectory value);
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
  boolean getCheckFullTrajectory();
  void setCheckFullTrajectory(boolean value);
  arm_navigation_msgs.Constraints getPathConstraints();
  void setPathConstraints(arm_navigation_msgs.Constraints value);
  arm_navigation_msgs.Constraints getGoalConstraints();
  void setGoalConstraints(arm_navigation_msgs.Constraints value);
}

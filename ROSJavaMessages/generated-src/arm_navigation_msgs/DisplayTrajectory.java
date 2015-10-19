package arm_navigation_msgs;

public interface DisplayTrajectory extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/DisplayTrajectory";
  static final java.lang.String _DEFINITION = "# The model id for which this path has been generated\nstring model_id\n# The representation of the path contains position values for all the joints that are moving along the path\narm_navigation_msgs/RobotTrajectory trajectory\n# The robot state is used to obtain positions for all/some of the joints of the robot. \n# It is used by the path display node to determine the positions of the joints that are not specified in the joint path message above. \n# If the robot state message contains joint position information for joints that are also mentioned in the joint path message, the positions in the joint path message will overwrite the positions specified in the robot state message. \nRobotState robot_state\n";
  java.lang.String getModelId();
  void setModelId(java.lang.String value);
  arm_navigation_msgs.RobotTrajectory getTrajectory();
  void setTrajectory(arm_navigation_msgs.RobotTrajectory value);
  arm_navigation_msgs.RobotState getRobotState();
  void setRobotState(arm_navigation_msgs.RobotState value);
}

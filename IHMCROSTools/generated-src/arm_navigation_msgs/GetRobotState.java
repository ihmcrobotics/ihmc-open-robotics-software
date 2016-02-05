package arm_navigation_msgs;

public interface GetRobotState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/GetRobotState";
  static final java.lang.String _DEFINITION = "# This message returns the current robot state from a node that has information\n# about the current state of the robot\n\n---\n\n# The returned robot_state contains the current state of the robot\narm_navigation_msgs/RobotState robot_state\n\n# Integer error code corresponding to the first check that was violated\n# The message contains both the returned error code value and a set \n# of possible error codes\narm_navigation_msgs/ArmNavigationErrorCodes error_code\n";
}

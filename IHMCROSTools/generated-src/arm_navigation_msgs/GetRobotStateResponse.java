package arm_navigation_msgs;

public interface GetRobotStateResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/GetRobotStateResponse";
  static final java.lang.String _DEFINITION = "\n# The returned robot_state contains the current state of the robot\narm_navigation_msgs/RobotState robot_state\n\n# Integer error code corresponding to the first check that was violated\n# The message contains both the returned error code value and a set \n# of possible error codes\narm_navigation_msgs/ArmNavigationErrorCodes error_code";
  arm_navigation_msgs.RobotState getRobotState();
  void setRobotState(arm_navigation_msgs.RobotState value);
  arm_navigation_msgs.ArmNavigationErrorCodes getErrorCode();
  void setErrorCode(arm_navigation_msgs.ArmNavigationErrorCodes value);
}

package kinematics_msgs;

public interface GetPositionIKResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinematics_msgs/GetPositionIKResponse";
  static final java.lang.String _DEFINITION = "# The returned solution \n# (in the same order as the list of joints specified in the IKRequest message)\narm_navigation_msgs/RobotState solution\n\narm_navigation_msgs/ArmNavigationErrorCodes error_code";
  arm_navigation_msgs.RobotState getSolution();
  void setSolution(arm_navigation_msgs.RobotState value);
  arm_navigation_msgs.ArmNavigationErrorCodes getErrorCode();
  void setErrorCode(arm_navigation_msgs.ArmNavigationErrorCodes value);
}

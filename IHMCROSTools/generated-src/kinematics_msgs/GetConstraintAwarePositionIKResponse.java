package kinematics_msgs;

public interface GetConstraintAwarePositionIKResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinematics_msgs/GetConstraintAwarePositionIKResponse";
  static final java.lang.String _DEFINITION = "# The returned solution \narm_navigation_msgs/RobotState solution\narm_navigation_msgs/ArmNavigationErrorCodes error_code";
  arm_navigation_msgs.RobotState getSolution();
  void setSolution(arm_navigation_msgs.RobotState value);
  arm_navigation_msgs.ArmNavigationErrorCodes getErrorCode();
  void setErrorCode(arm_navigation_msgs.ArmNavigationErrorCodes value);
}

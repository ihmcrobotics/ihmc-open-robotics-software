package arm_navigation_msgs;

public interface FilterJointTrajectoryWithConstraintsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/FilterJointTrajectoryWithConstraintsResponse";
  static final java.lang.String _DEFINITION = "trajectory_msgs/JointTrajectory trajectory\nArmNavigationErrorCodes error_code";
  trajectory_msgs.JointTrajectory getTrajectory();
  void setTrajectory(trajectory_msgs.JointTrajectory value);
  arm_navigation_msgs.ArmNavigationErrorCodes getErrorCode();
  void setErrorCode(arm_navigation_msgs.ArmNavigationErrorCodes value);
}

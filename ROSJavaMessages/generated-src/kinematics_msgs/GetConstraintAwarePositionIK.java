package kinematics_msgs;

public interface GetConstraintAwarePositionIK extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinematics_msgs/GetConstraintAwarePositionIK";
  static final java.lang.String _DEFINITION = "# A service call to carry out an inverse kinematics computation\n# The inverse kinematics request\nkinematics_msgs/PositionIKRequest ik_request\n# A set of constraints that the IK must obey\narm_navigation_msgs/Constraints constraints\n# Maximum allowed time for IK calculation\nduration timeout\n---\n# The returned solution \narm_navigation_msgs/RobotState solution\narm_navigation_msgs/ArmNavigationErrorCodes error_code\n";
}

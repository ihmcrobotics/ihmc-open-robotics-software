package kinematics_msgs;

public interface GetPositionIK extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinematics_msgs/GetPositionIK";
  static final java.lang.String _DEFINITION = "# A service call to carry out an inverse kinematics computation\n# The inverse kinematics request\nkinematics_msgs/PositionIKRequest ik_request\n# Maximum allowed time for IK calculation\nduration timeout\n---\n# The returned solution \n# (in the same order as the list of joints specified in the IKRequest message)\narm_navigation_msgs/RobotState solution\n\narm_navigation_msgs/ArmNavigationErrorCodes error_code\n\n";
}

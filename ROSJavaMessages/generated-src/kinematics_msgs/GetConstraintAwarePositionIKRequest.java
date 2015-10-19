package kinematics_msgs;

public interface GetConstraintAwarePositionIKRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinematics_msgs/GetConstraintAwarePositionIKRequest";
  static final java.lang.String _DEFINITION = "# A service call to carry out an inverse kinematics computation\n# The inverse kinematics request\nkinematics_msgs/PositionIKRequest ik_request\n# A set of constraints that the IK must obey\narm_navigation_msgs/Constraints constraints\n# Maximum allowed time for IK calculation\nduration timeout\n";
  kinematics_msgs.PositionIKRequest getIkRequest();
  void setIkRequest(kinematics_msgs.PositionIKRequest value);
  arm_navigation_msgs.Constraints getConstraints();
  void setConstraints(arm_navigation_msgs.Constraints value);
  org.ros.message.Duration getTimeout();
  void setTimeout(org.ros.message.Duration value);
}

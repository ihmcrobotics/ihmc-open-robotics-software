package kinematics_msgs;

public interface GetPositionIKRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinematics_msgs/GetPositionIKRequest";
  static final java.lang.String _DEFINITION = "# A service call to carry out an inverse kinematics computation\n# The inverse kinematics request\nkinematics_msgs/PositionIKRequest ik_request\n# Maximum allowed time for IK calculation\nduration timeout\n";
  kinematics_msgs.PositionIKRequest getIkRequest();
  void setIkRequest(kinematics_msgs.PositionIKRequest value);
  org.ros.message.Duration getTimeout();
  void setTimeout(org.ros.message.Duration value);
}

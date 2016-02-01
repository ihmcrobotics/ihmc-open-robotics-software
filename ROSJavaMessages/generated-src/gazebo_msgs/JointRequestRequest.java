package gazebo_msgs;

public interface JointRequestRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/JointRequestRequest";
  static final java.lang.String _DEFINITION = "string joint_name   # name of the joint requested\n";
  java.lang.String getJointName();
  void setJointName(java.lang.String value);
}

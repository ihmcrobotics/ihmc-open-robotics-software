package gazebo;

public interface JointRequestRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/JointRequestRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring joint_name   # name of the joint requested\n";
  java.lang.String getJointName();
  void setJointName(java.lang.String value);
}

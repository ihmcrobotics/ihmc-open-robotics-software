package gazebo_msgs;

public interface GetJointPropertiesRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/GetJointPropertiesRequest";
  static final java.lang.String _DEFINITION = "string joint_name                    # name of joint\n";
  java.lang.String getJointName();
  void setJointName(java.lang.String value);
}

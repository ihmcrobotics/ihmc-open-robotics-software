package gazebo;

public interface GetJointPropertiesRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/GetJointPropertiesRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring joint_name                    # name of joint\n";
  java.lang.String getJointName();
  void setJointName(java.lang.String value);
}

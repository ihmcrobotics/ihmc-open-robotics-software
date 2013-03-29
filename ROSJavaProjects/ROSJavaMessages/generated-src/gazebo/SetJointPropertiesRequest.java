package gazebo;

public interface SetJointPropertiesRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/SetJointPropertiesRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring joint_name                               # name of joint\ngazebo/ODEJointProperties ode_joint_config # access to ODE joint dynamics properties\n";
  java.lang.String getJointName();
  void setJointName(java.lang.String value);
  gazebo.ODEJointProperties getOdeJointConfig();
  void setOdeJointConfig(gazebo.ODEJointProperties value);
}

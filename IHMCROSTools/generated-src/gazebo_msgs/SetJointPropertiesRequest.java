package gazebo_msgs;

public interface SetJointPropertiesRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/SetJointPropertiesRequest";
  static final java.lang.String _DEFINITION = "string joint_name                               # name of joint\ngazebo_msgs/ODEJointProperties ode_joint_config # access to ODE joint dynamics properties\n";
  java.lang.String getJointName();
  void setJointName(java.lang.String value);
  gazebo_msgs.ODEJointProperties getOdeJointConfig();
  void setOdeJointConfig(gazebo_msgs.ODEJointProperties value);
}

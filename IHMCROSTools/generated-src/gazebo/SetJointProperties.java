package gazebo;

public interface SetJointProperties extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/SetJointProperties";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring joint_name                               # name of joint\ngazebo/ODEJointProperties ode_joint_config # access to ODE joint dynamics properties\n---\nbool success                                    # return true if get successful\nstring status_message                           # comments if available\n";
}

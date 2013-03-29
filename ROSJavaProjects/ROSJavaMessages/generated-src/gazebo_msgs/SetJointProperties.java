package gazebo_msgs;

public interface SetJointProperties extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/SetJointProperties";
  static final java.lang.String _DEFINITION = "string joint_name                               # name of joint\ngazebo_msgs/ODEJointProperties ode_joint_config # access to ODE joint dynamics properties\n---\nbool success                                    # return true if get successful\nstring status_message                           # comments if available\n";
}

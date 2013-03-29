package gazebo_msgs;

public interface SetPhysicsProperties extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/SetPhysicsProperties";
  static final java.lang.String _DEFINITION = "# sets pose and twist of a link.  All children link poses/twists of the URDF tree will be updated accordingly\nfloat64 time_step                  # dt in seconds\nfloat64 max_update_rate            # throttle maximum physics update rate\ngeometry_msgs/Vector3 gravity      # gravity vector (e.g. earth ~[0,0,-9.81])\ngazebo_msgs/ODEPhysics ode_config  # configurations for ODE\n---\nbool success                       # return true if set wrench successful\nstring status_message              # comments if available\n";
}

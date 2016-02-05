package gazebo;

public interface GetPhysicsProperties extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/GetPhysicsProperties";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\n---\n# sets pose and twist of a link.  All children link poses/twists of the URDF tree will be updated accordingly\nfloat64 time_step                  # dt in seconds\nbool pause                         # true if physics engine is paused\nfloat64 max_update_rate            # throttle maximum physics update rate\ngeometry_msgs/Vector3 gravity      # gravity vector (e.g. earth ~[0,0,-9.81])\ngazebo/ODEPhysics ode_config  # contains physics configurations pertaining to ODE\nbool success                       # return true if set wrench successful\nstring status_message              # comments if available\n";
}

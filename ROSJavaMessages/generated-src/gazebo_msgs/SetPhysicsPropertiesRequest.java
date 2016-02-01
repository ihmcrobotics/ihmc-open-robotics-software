package gazebo_msgs;

public interface SetPhysicsPropertiesRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/SetPhysicsPropertiesRequest";
  static final java.lang.String _DEFINITION = "# sets pose and twist of a link.  All children link poses/twists of the URDF tree will be updated accordingly\nfloat64 time_step                  # dt in seconds\nfloat64 max_update_rate            # throttle maximum physics update rate\ngeometry_msgs/Vector3 gravity      # gravity vector (e.g. earth ~[0,0,-9.81])\ngazebo_msgs/ODEPhysics ode_config  # configurations for ODE\n";
  double getTimeStep();
  void setTimeStep(double value);
  double getMaxUpdateRate();
  void setMaxUpdateRate(double value);
  geometry_msgs.Vector3 getGravity();
  void setGravity(geometry_msgs.Vector3 value);
  gazebo_msgs.ODEPhysics getOdeConfig();
  void setOdeConfig(gazebo_msgs.ODEPhysics value);
}

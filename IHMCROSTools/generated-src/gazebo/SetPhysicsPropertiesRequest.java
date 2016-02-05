package gazebo;

public interface SetPhysicsPropertiesRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/SetPhysicsPropertiesRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\n# sets pose and twist of a link.  All children link poses/twists of the URDF tree will be updated accordingly\nfloat64 time_step                  # dt in seconds\nbool pause                         # pause physics\nfloat64 max_update_rate            # throttle maximum physics update rate\ngeometry_msgs/Vector3 gravity      # gravity vector (e.g. earth ~[0,0,-9.81])\ngazebo/ODEPhysics ode_config  # configurations for ODE\n";
  double getTimeStep();
  void setTimeStep(double value);
  boolean getPause();
  void setPause(boolean value);
  double getMaxUpdateRate();
  void setMaxUpdateRate(double value);
  geometry_msgs.Vector3 getGravity();
  void setGravity(geometry_msgs.Vector3 value);
  gazebo.ODEPhysics getOdeConfig();
  void setOdeConfig(gazebo.ODEPhysics value);
}

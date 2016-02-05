package gazebo;

public interface GetPhysicsPropertiesResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/GetPhysicsPropertiesResponse";
  static final java.lang.String _DEFINITION = "# sets pose and twist of a link.  All children link poses/twists of the URDF tree will be updated accordingly\nfloat64 time_step                  # dt in seconds\nbool pause                         # true if physics engine is paused\nfloat64 max_update_rate            # throttle maximum physics update rate\ngeometry_msgs/Vector3 gravity      # gravity vector (e.g. earth ~[0,0,-9.81])\ngazebo/ODEPhysics ode_config  # contains physics configurations pertaining to ODE\nbool success                       # return true if set wrench successful\nstring status_message              # comments if available";
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
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}

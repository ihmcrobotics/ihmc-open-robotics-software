package gazebo;

public interface SetModelConfiguration extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/SetModelConfiguration";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\n# Set Gazebo Model pose and twist\nstring model_name           # model to set state (pose and twist)\nstring test_urdf_param_name # parameter name that contains the urdf XML. (this is deprecated, new msg has no test_prefix.)\n\nstring[] joint_names        # list of joints to set positions.  if joint is not listed here, preserve current position.\nfloat64[] joint_positions   # set to this position.\n---\nbool success                # return true if setting state successful\nstring status_message       # comments if available\n";
}

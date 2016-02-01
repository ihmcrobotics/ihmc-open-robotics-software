package gazebo_msgs;

public interface SetModelConfiguration extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/SetModelConfiguration";
  static final java.lang.String _DEFINITION = "# Set Gazebo Model pose and twist\nstring model_name           # model to set state (pose and twist)\nstring urdf_param_name      # parameter name that contains the urdf XML.\n\nstring[] joint_names        # list of joints to set positions.  if joint is not listed here, preserve current position.\nfloat64[] joint_positions   # set to this position.\n---\nbool success                # return true if setting state successful\nstring status_message       # comments if available\n";
}

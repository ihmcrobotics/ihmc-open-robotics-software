package gazebo;

public interface SetModelState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/SetModelState";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\ngazebo/ModelState model_state\n---\nbool success                  # return true if setting state successful\nstring status_message         # comments if available\n";
}

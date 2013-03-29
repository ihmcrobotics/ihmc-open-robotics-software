package gazebo;

public interface DeleteModel extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/DeleteModel";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring model_name                 # name of the Gazebo Model to be deleted\n---\nbool success                      # return true if deletion is successful\nstring status_message             # comments if available\n";
}

package gazebo_msgs;

public interface DeleteModel extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/DeleteModel";
  static final java.lang.String _DEFINITION = "string model_name                 # name of the Gazebo Model to be deleted\n---\nbool success                      # return true if deletion is successful\nstring status_message             # comments if available\n";
}

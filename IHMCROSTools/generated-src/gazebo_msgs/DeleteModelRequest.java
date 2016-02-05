package gazebo_msgs;

public interface DeleteModelRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/DeleteModelRequest";
  static final java.lang.String _DEFINITION = "string model_name                 # name of the Gazebo Model to be deleted\n";
  java.lang.String getModelName();
  void setModelName(java.lang.String value);
}

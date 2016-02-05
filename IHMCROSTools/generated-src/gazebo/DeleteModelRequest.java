package gazebo;

public interface DeleteModelRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/DeleteModelRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring model_name                 # name of the Gazebo Model to be deleted\n";
  java.lang.String getModelName();
  void setModelName(java.lang.String value);
}

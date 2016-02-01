package gazebo;

public interface SetModelStateRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/SetModelStateRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\ngazebo/ModelState model_state\n";
  gazebo.ModelState getModelState();
  void setModelState(gazebo.ModelState value);
}

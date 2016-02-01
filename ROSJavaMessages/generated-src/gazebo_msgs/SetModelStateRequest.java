package gazebo_msgs;

public interface SetModelStateRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/SetModelStateRequest";
  static final java.lang.String _DEFINITION = "gazebo_msgs/ModelState model_state\n";
  gazebo_msgs.ModelState getModelState();
  void setModelState(gazebo_msgs.ModelState value);
}

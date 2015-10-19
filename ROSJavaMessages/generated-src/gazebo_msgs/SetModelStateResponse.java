package gazebo_msgs;

public interface SetModelStateResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/SetModelStateResponse";
  static final java.lang.String _DEFINITION = "bool success                  # return true if setting state successful\nstring status_message         # comments if available";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}

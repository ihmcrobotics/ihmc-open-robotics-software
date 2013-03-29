package gazebo_msgs;

public interface DeleteModelResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/DeleteModelResponse";
  static final java.lang.String _DEFINITION = "bool success                      # return true if deletion is successful\nstring status_message             # comments if available";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}

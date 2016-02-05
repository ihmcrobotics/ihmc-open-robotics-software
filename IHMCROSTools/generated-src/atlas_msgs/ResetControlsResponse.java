package atlas_msgs;

public interface ResetControlsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/ResetControlsResponse";
  static final java.lang.String _DEFINITION = "bool success\nstring status_message";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}

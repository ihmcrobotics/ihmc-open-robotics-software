package sensor_msgs;

public interface SetCameraInfoResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_msgs/SetCameraInfoResponse";
  static final java.lang.String _DEFINITION = "bool success          # True if the call succeeded\nstring status_message # Used to give details about success";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}

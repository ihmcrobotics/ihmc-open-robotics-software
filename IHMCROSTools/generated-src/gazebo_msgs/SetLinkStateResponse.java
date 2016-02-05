package gazebo_msgs;

public interface SetLinkStateResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/SetLinkStateResponse";
  static final java.lang.String _DEFINITION = "bool success                # return true if set wrench successful\nstring status_message       # comments if available";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}

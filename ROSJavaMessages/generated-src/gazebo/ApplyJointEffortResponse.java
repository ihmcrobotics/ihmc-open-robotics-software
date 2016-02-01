package gazebo;

public interface ApplyJointEffortResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/ApplyJointEffortResponse";
  static final java.lang.String _DEFINITION = "bool success                # return true if effort application is successful\nstring status_message       # comments if available";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}

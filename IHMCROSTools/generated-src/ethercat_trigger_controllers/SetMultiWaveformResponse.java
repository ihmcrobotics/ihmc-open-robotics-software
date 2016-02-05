package ethercat_trigger_controllers;

public interface SetMultiWaveformResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethercat_trigger_controllers/SetMultiWaveformResponse";
  static final java.lang.String _DEFINITION = "bool success\nstring status_message";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}

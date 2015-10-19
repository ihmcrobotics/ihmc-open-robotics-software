package app_manager;

public interface StopAppResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/StopAppResponse";
  static final java.lang.String _DEFINITION = "# true if app stopped, false otherwise\nbool stopped\n# if app did not stop, error code for classifying stop failure.  See\n# StatusCodes.msg for common codes.\nint32 error_code\nstring message";
  boolean getStopped();
  void setStopped(boolean value);
  int getErrorCode();
  void setErrorCode(int value);
  java.lang.String getMessage();
  void setMessage(java.lang.String value);
}

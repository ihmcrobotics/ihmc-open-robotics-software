package app_manager;

public interface StartAppResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/StartAppResponse";
  static final java.lang.String _DEFINITION = "# true if app started, false otherwise\nbool started\n# if app did not start, error code for classifying start failure.  See\n# StatusCodes.msg for common codes.\nint32 error_code\n# response message for debugging\nstring message\n# Namespace where the app interface can be found\nstring namespace  ";
  boolean getStarted();
  void setStarted(boolean value);
  int getErrorCode();
  void setErrorCode(int value);
  java.lang.String getMessage();
  void setMessage(java.lang.String value);
  java.lang.String getNamespace();
  void setNamespace(java.lang.String value);
}

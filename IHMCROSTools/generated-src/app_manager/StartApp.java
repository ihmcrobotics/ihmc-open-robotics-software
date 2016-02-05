package app_manager;

public interface StartApp extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/StartApp";
  static final java.lang.String _DEFINITION = "# Name of the app to launch\nstring name \n---\n# true if app started, false otherwise\nbool started\n# if app did not start, error code for classifying start failure.  See\n# StatusCodes.msg for common codes.\nint32 error_code\n# response message for debugging\nstring message\n# Namespace where the app interface can be found\nstring namespace  \n\n";
}

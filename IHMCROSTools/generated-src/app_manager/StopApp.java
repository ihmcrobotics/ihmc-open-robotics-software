package app_manager;

public interface StopApp extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/StopApp";
  static final java.lang.String _DEFINITION = "# Name of app to stop.  Sending \"*\" stops all apps.\nstring name\n---\n# true if app stopped, false otherwise\nbool stopped\n# if app did not stop, error code for classifying stop failure.  See\n# StatusCodes.msg for common codes.\nint32 error_code\nstring message\n";
}

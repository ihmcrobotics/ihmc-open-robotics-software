package app_manager;

public interface InstallApp extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/InstallApp";
  static final java.lang.String _DEFINITION = "# Name of the app to install or upgrade\nstring name \n---\n# true if app started, false otherwise\nbool installed\n# response message for debugging\nstring message\n\n";
}

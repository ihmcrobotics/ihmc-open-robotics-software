package app_manager;

public interface InstallAppResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/InstallAppResponse";
  static final java.lang.String _DEFINITION = "# true if app started, false otherwise\nbool installed\n# response message for debugging\nstring message";
  boolean getInstalled();
  void setInstalled(boolean value);
  java.lang.String getMessage();
  void setMessage(java.lang.String value);
}

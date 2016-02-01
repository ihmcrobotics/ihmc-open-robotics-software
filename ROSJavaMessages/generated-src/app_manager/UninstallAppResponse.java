package app_manager;

public interface UninstallAppResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/UninstallAppResponse";
  static final java.lang.String _DEFINITION = "# true if app stopped, false otherwise\nbool uninstalled\nstring message";
  boolean getUninstalled();
  void setUninstalled(boolean value);
  java.lang.String getMessage();
  void setMessage(java.lang.String value);
}

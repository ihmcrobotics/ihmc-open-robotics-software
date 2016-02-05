package app_manager;

public interface UninstallAppRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/UninstallAppRequest";
  static final java.lang.String _DEFINITION = "# Name of app to uninstall\nstring name\n";
  java.lang.String getName();
  void setName(java.lang.String value);
}

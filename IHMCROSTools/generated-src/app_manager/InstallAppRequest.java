package app_manager;

public interface InstallAppRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/InstallAppRequest";
  static final java.lang.String _DEFINITION = "# Name of the app to install or upgrade\nstring name \n";
  java.lang.String getName();
  void setName(java.lang.String value);
}

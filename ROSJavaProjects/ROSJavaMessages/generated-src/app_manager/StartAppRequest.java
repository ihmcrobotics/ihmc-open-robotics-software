package app_manager;

public interface StartAppRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/StartAppRequest";
  static final java.lang.String _DEFINITION = "# Name of the app to launch\nstring name \n";
  java.lang.String getName();
  void setName(java.lang.String value);
}

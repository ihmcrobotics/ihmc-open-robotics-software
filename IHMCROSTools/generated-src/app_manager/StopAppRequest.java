package app_manager;

public interface StopAppRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/StopAppRequest";
  static final java.lang.String _DEFINITION = "# Name of app to stop.  Sending \"*\" stops all apps.\nstring name\n";
  java.lang.String getName();
  void setName(java.lang.String value);
}

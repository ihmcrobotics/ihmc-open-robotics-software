package app_manager;

public interface AppStatus extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/AppStatus";
  static final java.lang.String _DEFINITION = "int32 INFO=0\nint32 WARN=1\nint32 ERROR=2\n# Status type.  One of INFO, WARN, ERROR.\nint32 type\n# Status message.\nstring status\n";
  static final int INFO = 0;
  static final int WARN = 1;
  static final int ERROR = 2;
  int getType();
  void setType(int value);
  java.lang.String getStatus();
  void setStatus(java.lang.String value);
}

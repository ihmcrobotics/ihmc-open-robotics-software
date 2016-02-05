package roscpp;

public interface SetLoggerLevelRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "roscpp/SetLoggerLevelRequest";
  static final java.lang.String _DEFINITION = "string logger\nstring level\n";
  java.lang.String getLogger();
  void setLogger(java.lang.String value);
  java.lang.String getLevel();
  void setLevel(java.lang.String value);
}

package roscpp;

public interface Logger extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "roscpp/Logger";
  static final java.lang.String _DEFINITION = "string name\nstring level\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getLevel();
  void setLevel(java.lang.String value);
}

package turtlesim;

public interface KillRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "turtlesim/KillRequest";
  static final java.lang.String _DEFINITION = "string name\n";
  java.lang.String getName();
  void setName(java.lang.String value);
}

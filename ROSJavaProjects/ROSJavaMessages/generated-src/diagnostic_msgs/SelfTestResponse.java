package diagnostic_msgs;

public interface SelfTestResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "diagnostic_msgs/SelfTestResponse";
  static final java.lang.String _DEFINITION = "string id\nbyte passed\nDiagnosticStatus[] status";
  java.lang.String getId();
  void setId(java.lang.String value);
  byte getPassed();
  void setPassed(byte value);
  java.util.List<diagnostic_msgs.DiagnosticStatus> getStatus();
  void setStatus(java.util.List<diagnostic_msgs.DiagnosticStatus> value);
}

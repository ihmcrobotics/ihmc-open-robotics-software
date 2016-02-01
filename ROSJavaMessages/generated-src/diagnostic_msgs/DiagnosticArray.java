package diagnostic_msgs;

public interface DiagnosticArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "diagnostic_msgs/DiagnosticArray";
  static final java.lang.String _DEFINITION = "# This message is used to send diagnostic information about the state of the robot\nHeader header #for timestamp\nDiagnosticStatus[] status # an array of components being reported on";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<diagnostic_msgs.DiagnosticStatus> getStatus();
  void setStatus(java.util.List<diagnostic_msgs.DiagnosticStatus> value);
}

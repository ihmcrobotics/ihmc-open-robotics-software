package foothold_finding_msg;

public interface AdaptFootholdsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "foothold_finding_msg/AdaptFootholdsResponse";
  static final java.lang.String _DEFINITION = "\r\nFoothold[] adaptedFoodholds\r";
  java.util.List<foothold_finding_msg.Foothold> getAdaptedFoodholds();
  void setAdaptedFoodholds(java.util.List<foothold_finding_msg.Foothold> value);
}

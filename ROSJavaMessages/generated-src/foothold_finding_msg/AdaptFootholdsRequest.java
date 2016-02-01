package foothold_finding_msg;

public interface AdaptFootholdsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "foothold_finding_msg/AdaptFootholdsRequest";
  static final java.lang.String _DEFINITION = "# Takes an initial list of footholds and returns the footholds adapted to the terrain.\r\n\r\nFoothold[] initialFootholds\r\n\r\n";
  java.util.List<foothold_finding_msg.Foothold> getInitialFootholds();
  void setInitialFootholds(java.util.List<foothold_finding_msg.Foothold> value);
}

package foothold_adaptation_msg;

public interface AdaptFootholdsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "foothold_adaptation_msg/AdaptFootholdsRequest";
  static final java.lang.String _DEFINITION = "# Takes an initial list of footholds and returns the footholds adapted to the terrain\r\n\r\nFootholdList initialFoodholds\r\n";
  foothold_adaptation_msg.FootholdList getInitialFoodholds();
  void setInitialFoodholds(foothold_adaptation_msg.FootholdList value);
}

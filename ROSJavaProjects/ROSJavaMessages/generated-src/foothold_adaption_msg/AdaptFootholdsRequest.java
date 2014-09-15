package foothold_adaption_msg;

public interface AdaptFootholdsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "foothold_adaption_msg/AdaptFootholdsRequest";
  static final java.lang.String _DEFINITION = "# Takes an initial list of footholds and returns the footholds adapted to the terrain\r\n\r\nFootholdList initialFoodholds\r\n";
  foothold_adaption_msg.FootholdList getInitialFoodholds();
  void setInitialFoodholds(foothold_adaption_msg.FootholdList value);
}

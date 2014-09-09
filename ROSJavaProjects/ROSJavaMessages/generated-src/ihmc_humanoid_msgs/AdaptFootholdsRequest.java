package ihmc_humanoid_msgs;

public interface AdaptFootholdsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_humanoid_msgs/AdaptFootholdsRequest";
  static final java.lang.String _DEFINITION = "# Takes an initial list of footholds and returns the footholds adapted to the terrain\r\n\r\nFootholdList initialFoodholds\r\n";
  ihmc_humanoid_msgs.FootholdList getInitialFoodholds();
  void setInitialFoodholds(ihmc_humanoid_msgs.FootholdList value);
}

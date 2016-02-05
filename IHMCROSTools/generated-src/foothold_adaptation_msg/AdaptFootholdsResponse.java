package foothold_adaptation_msg;

public interface AdaptFootholdsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "foothold_adaptation_msg/AdaptFootholdsResponse";
  static final java.lang.String _DEFINITION = "FootholdList adaptedFootholds";
  foothold_adaptation_msg.FootholdList getAdaptedFootholds();
  void setAdaptedFootholds(foothold_adaptation_msg.FootholdList value);
}

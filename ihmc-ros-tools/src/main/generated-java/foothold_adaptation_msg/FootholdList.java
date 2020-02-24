package foothold_adaptation_msg;

public interface FootholdList extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "foothold_adaptation_msg/FootholdList";
  static final java.lang.String _DEFINITION = "# List of footholds, seq number in the header defines order of footholds\r\nFoothold[] footholds";
  java.util.List<foothold_adaptation_msg.Foothold> getFootholds();
  void setFootholds(java.util.List<foothold_adaptation_msg.Foothold> value);
}

package multisense_ros;

public interface StampedPps extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "multisense_ros/StampedPps";
  static final java.lang.String _DEFINITION = "time     data\ntime     host_time\n";
  org.ros.message.Time getData();
  void setData(org.ros.message.Time value);
  org.ros.message.Time getHostTime();
  void setHostTime(org.ros.message.Time value);
}

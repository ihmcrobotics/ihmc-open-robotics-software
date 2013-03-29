package sensor_msgs;

public interface TimeReference extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_msgs/TimeReference";
  static final java.lang.String _DEFINITION = "# Measurement from an external time source not actively synchronized with the system clock.\n\nHeader header    # stamp is system time for which measurement was valid\n                 # frame_id is not used \n\ntime   time_ref  # corresponding time from this external source\nstring source    # (optional) name of time source\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  org.ros.message.Time getTimeRef();
  void setTimeRef(org.ros.message.Time value);
  java.lang.String getSource();
  void setSource(java.lang.String value);
}

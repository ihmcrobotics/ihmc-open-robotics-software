package bond;

public interface Status extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "bond/Status";
  static final java.lang.String _DEFINITION = "Header header\nstring id  # ID of the bond\nstring instance_id  # Unique ID for an individual in a bond\nbool active\n\n# Including the timeouts for the bond makes it easier to debug mis-matches\n# between the two sides.\nfloat32 heartbeat_timeout\nfloat32 heartbeat_period";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getId();
  void setId(java.lang.String value);
  java.lang.String getInstanceId();
  void setInstanceId(java.lang.String value);
  boolean getActive();
  void setActive(boolean value);
  float getHeartbeatTimeout();
  void setHeartbeatTimeout(float value);
  float getHeartbeatPeriod();
  void setHeartbeatPeriod(float value);
}

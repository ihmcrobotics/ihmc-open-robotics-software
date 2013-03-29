package diagnostic_msgs;

public interface KeyValue extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "diagnostic_msgs/KeyValue";
  static final java.lang.String _DEFINITION = "string key # what to label this value when viewing\nstring value # a value to track over time\n";
  java.lang.String getKey();
  void setKey(java.lang.String value);
  java.lang.String getValue();
  void setValue(java.lang.String value);
}

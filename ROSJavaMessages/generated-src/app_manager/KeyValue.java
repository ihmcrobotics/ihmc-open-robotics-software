package app_manager;

public interface KeyValue extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/KeyValue";
  static final java.lang.String _DEFINITION = "string key\nstring value\n";
  java.lang.String getKey();
  void setKey(java.lang.String value);
  java.lang.String getValue();
  void setValue(java.lang.String value);
}

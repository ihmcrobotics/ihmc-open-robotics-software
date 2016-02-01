package driver_base;

public interface ConfigString extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "driver_base/ConfigString";
  static final java.lang.String _DEFINITION = "string name\nstring value\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getValue();
  void setValue(java.lang.String value);
}

package driver_base;

public interface ConfigValue extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "driver_base/ConfigValue";
  static final java.lang.String _DEFINITION = "string name\nfloat64 value\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  double getValue();
  void setValue(double value);
}

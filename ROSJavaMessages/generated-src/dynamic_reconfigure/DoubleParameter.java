package dynamic_reconfigure;

public interface DoubleParameter extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "dynamic_reconfigure/DoubleParameter";
  static final java.lang.String _DEFINITION = "string name\nfloat64 value\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  double getValue();
  void setValue(double value);
}

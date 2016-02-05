package dynamic_reconfigure;

public interface IntParameter extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "dynamic_reconfigure/IntParameter";
  static final java.lang.String _DEFINITION = "string name\nint32 value\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  int getValue();
  void setValue(int value);
}

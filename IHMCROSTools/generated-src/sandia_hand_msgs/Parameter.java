package sandia_hand_msgs;

public interface Parameter extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sandia_hand_msgs/Parameter";
  static final java.lang.String _DEFINITION = "string  name\nbyte    INTEGER=1\nbyte    FLOAT=2\nbyte    val_type\nuint32  i_val\nfloat32 f_val\n";
  static final byte INTEGER = 1;
  static final byte FLOAT = 2;
  java.lang.String getName();
  void setName(java.lang.String value);
  byte getValType();
  void setValType(byte value);
  int getIVal();
  void setIVal(int value);
  float getFVal();
  void setFVal(float value);
}

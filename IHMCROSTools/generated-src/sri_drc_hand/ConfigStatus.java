package sri_drc_hand;

public interface ConfigStatus extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sri_drc_hand/ConfigStatus";
  static final java.lang.String _DEFINITION = "time stamp\nuint8 action\nuint8 result\nuint16 index\nfloat32 value\n";
  org.ros.message.Time getStamp();
  void setStamp(org.ros.message.Time value);
  byte getAction();
  void setAction(byte value);
  byte getResult();
  void setResult(byte value);
  short getIndex();
  void setIndex(short value);
  float getValue();
  void setValue(float value);
}

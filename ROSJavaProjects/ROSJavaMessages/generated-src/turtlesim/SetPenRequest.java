package turtlesim;

public interface SetPenRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "turtlesim/SetPenRequest";
  static final java.lang.String _DEFINITION = "uint8 r\nuint8 g\nuint8 b\nuint8 width\nuint8 off\n";
  byte getR();
  void setR(byte value);
  byte getG();
  void setG(byte value);
  byte getB();
  void setB(byte value);
  byte getWidth();
  void setWidth(byte value);
  byte getOff();
  void setOff(byte value);
}

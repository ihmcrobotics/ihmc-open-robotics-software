package turtlesim;

public interface Color extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "turtlesim/Color";
  static final java.lang.String _DEFINITION = "uint8 r\nuint8 g\nuint8 b\n";
  byte getR();
  void setR(byte value);
  byte getG();
  void setG(byte value);
  byte getB();
  void setB(byte value);
}

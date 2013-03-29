package test_ros;

public interface Simple extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_ros/Simple";
  static final java.lang.String _DEFINITION = "# for rostopic tests\nbyte b\nint16 int16\nint32 int32\nint64 int64\nchar c\nuint16 uint16\nuint32 uint32\nuint64 uint64\nstring str\n";
  byte getB();
  void setB(byte value);
  short getInt16();
  void setInt16(short value);
  int getInt32();
  void setInt32(int value);
  long getInt64();
  void setInt64(long value);
  byte getC();
  void setC(byte value);
  short getUint16();
  void setUint16(short value);
  int getUint32();
  void setUint32(int value);
  long getUint64();
  void setUint64(long value);
  java.lang.String getStr();
  void setStr(java.lang.String value);
}

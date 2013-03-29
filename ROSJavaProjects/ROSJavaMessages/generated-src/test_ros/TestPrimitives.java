package test_ros;

public interface TestPrimitives extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_ros/TestPrimitives";
  static final java.lang.String _DEFINITION = "# Integration test message of all primitive types\n\n# caller_id of most recent node to send this message\nstring caller_id\n# caller_id of the original node to send this message\nstring orig_caller_id\n\nstring str\nbyte b\nint16 int16\nint32 int32\nint64 int64\nchar c\nuint16 uint16\nuint32 uint32\nuint64 uint64\nfloat32 float32\nfloat64 float64\ntime t\nduration d\n\n";
  java.lang.String getCallerId();
  void setCallerId(java.lang.String value);
  java.lang.String getOrigCallerId();
  void setOrigCallerId(java.lang.String value);
  java.lang.String getStr();
  void setStr(java.lang.String value);
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
  float getFloat32();
  void setFloat32(float value);
  double getFloat64();
  void setFloat64(double value);
  org.ros.message.Time getT();
  void setT(org.ros.message.Time value);
  org.ros.message.Duration getD();
  void setD(org.ros.message.Duration value);
}

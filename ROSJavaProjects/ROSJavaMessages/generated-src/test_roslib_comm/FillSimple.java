package test_roslib_comm;

public interface FillSimple extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roslib_comm/FillSimple";
  static final java.lang.String _DEFINITION = "int32 i32\nstring str\nint32[] i32_array\nbool b";
  int getI32();
  void setI32(int value);
  java.lang.String getStr();
  void setStr(java.lang.String value);
  int[] getI32Array();
  void setI32Array(int[] value);
  boolean getB();
  void setB(boolean value);
}

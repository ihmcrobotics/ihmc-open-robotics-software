package test_rospy;

public interface ConstantsMultiplexResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/ConstantsMultiplexResponse";
  static final java.lang.String _DEFINITION = "# test response constants as well\nbyte CONFIRM_X=1\nbyte CONFIRM_Y=2\nbyte CONFIRM_Z=3\nbyte select_confirm\nbyte ret_byte\nint32 ret_int32\nuint32 ret_uint32\nfloat32 ret_float32";
  static final byte CONFIRM_X = 1;
  static final byte CONFIRM_Y = 2;
  static final byte CONFIRM_Z = 3;
  byte getSelectConfirm();
  void setSelectConfirm(byte value);
  byte getRetByte();
  void setRetByte(byte value);
  int getRetInt32();
  void setRetInt32(int value);
  int getRetUint32();
  void setRetUint32(int value);
  float getRetFloat32();
  void setRetFloat32(float value);
}

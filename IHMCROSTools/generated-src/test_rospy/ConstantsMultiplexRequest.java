package test_rospy;

public interface ConstantsMultiplexRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/ConstantsMultiplexRequest";
  static final java.lang.String _DEFINITION = "byte BYTE_X=0\nbyte BYTE_Y=15\nbyte BYTE_Z=5\nint32 INT32_X=0\nint32 INT32_Y=-12345678\nint32 INT32_Z=12345678\nuint32 UINT32_X=0\nuint32 UINT32_Y=12345678\nuint32 UINT32_Z=1\nfloat32 FLOAT32_X=0.0\nfloat32 FLOAT32_Y=-3.14159\nfloat32 FLOAT32_Z=12345.78\nbyte SELECT_X=1\nbyte SELECT_Y=2\nbyte SELECT_Z=3\nbyte selection\n";
  static final byte BYTE_X = 0;
  static final byte BYTE_Y = 15;
  static final byte BYTE_Z = 5;
  static final int INT32_X = 0;
  static final int INT32_Y = -12345678;
  static final int INT32_Z = 12345678;
  static final int UINT32_X = 0;
  static final int UINT32_Y = 12345678;
  static final int UINT32_Z = 1;
  static final float FLOAT32_X = 0.0f;
  static final float FLOAT32_Y = -3.14159f;
  static final float FLOAT32_Z = 12345.78f;
  static final byte SELECT_X = 1;
  static final byte SELECT_Y = 2;
  static final byte SELECT_Z = 3;
  byte getSelection();
  void setSelection(byte value);
}

package test_ros;

public interface Arrays extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_ros/Arrays";
  static final java.lang.String _DEFINITION = "#for rostopic tests\nint8[] int8_arr\nuint8[] uint8_arr\nint32[] int32_arr\nuint32[] uint32_arr\nstring[] string_arr\ntime[] time_arr\n";
  org.jboss.netty.buffer.ChannelBuffer getInt8Arr();
  void setInt8Arr(org.jboss.netty.buffer.ChannelBuffer value);
  org.jboss.netty.buffer.ChannelBuffer getUint8Arr();
  void setUint8Arr(org.jboss.netty.buffer.ChannelBuffer value);
  int[] getInt32Arr();
  void setInt32Arr(int[] value);
  int[] getUint32Arr();
  void setUint32Arr(int[] value);
  java.util.List<java.lang.String> getStringArr();
  void setStringArr(java.util.List<java.lang.String> value);
  java.util.List<org.ros.message.Time> getTimeArr();
  void setTimeArr(java.util.List<org.ros.message.Time> value);
}

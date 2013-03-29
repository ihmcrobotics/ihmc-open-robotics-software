package test_rospy;

public interface TestFixedArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rospy/TestFixedArray";
  static final java.lang.String _DEFINITION = "float32[1] f32_1\nfloat32[3] f32_3\nfloat64[1] f64_1\nfloat64[3] f64_3\nint8[1] i8_1\nint8[3] i8_3\nuint8[1] u8_1\nuint8[3] u8_3\nint32[1] i32_1\nint32[3] i32_3\nuint32[1] u32_1\nuint32[3] u32_3\nstring[1] s_1\nstring[3] s_3\nbool[1] b_1\nbool[3] b_3";
  float[] getF321();
  void setF321(float[] value);
  float[] getF323();
  void setF323(float[] value);
  double[] getF641();
  void setF641(double[] value);
  double[] getF643();
  void setF643(double[] value);
  org.jboss.netty.buffer.ChannelBuffer getI81();
  void setI81(org.jboss.netty.buffer.ChannelBuffer value);
  org.jboss.netty.buffer.ChannelBuffer getI83();
  void setI83(org.jboss.netty.buffer.ChannelBuffer value);
  org.jboss.netty.buffer.ChannelBuffer getU81();
  void setU81(org.jboss.netty.buffer.ChannelBuffer value);
  org.jboss.netty.buffer.ChannelBuffer getU83();
  void setU83(org.jboss.netty.buffer.ChannelBuffer value);
  int[] getI321();
  void setI321(int[] value);
  int[] getI323();
  void setI323(int[] value);
  int[] getU321();
  void setU321(int[] value);
  int[] getU323();
  void setU323(int[] value);
  java.util.List<java.lang.String> getS1();
  void setS1(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getS3();
  void setS3(java.util.List<java.lang.String> value);
  boolean[] getB1();
  void setB1(boolean[] value);
  boolean[] getB3();
  void setB3(boolean[] value);
}

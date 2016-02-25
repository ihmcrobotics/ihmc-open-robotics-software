package sandia_hand_msgs;

public interface RelativeJointCommands extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sandia_hand_msgs/RelativeJointCommands";
  static final java.lang.String _DEFINITION = "Header header\nfloat32[12] position\nuint8[12] max_effort\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float[] getPosition();
  void setPosition(float[] value);
  org.jboss.netty.buffer.ChannelBuffer getMaxEffort();
  void setMaxEffort(org.jboss.netty.buffer.ChannelBuffer value);
}

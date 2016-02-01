package navfn;

public interface SetCostmapRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "navfn/SetCostmapRequest";
  static final java.lang.String _DEFINITION = "uint8[] costs\nuint16 height\nuint16 width\n";
  org.jboss.netty.buffer.ChannelBuffer getCosts();
  void setCosts(org.jboss.netty.buffer.ChannelBuffer value);
  short getHeight();
  void setHeight(short value);
  short getWidth();
  void setWidth(short value);
}

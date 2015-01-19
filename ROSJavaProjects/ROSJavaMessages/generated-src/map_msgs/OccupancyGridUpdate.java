package map_msgs;

public interface OccupancyGridUpdate extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_msgs/OccupancyGridUpdate";
  static final java.lang.String _DEFINITION = "Header header\nint32 x\nint32 y\nuint32 width\nuint32 height\nint8[] data\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getX();
  void setX(int value);
  int getY();
  void setY(int value);
  int getWidth();
  void setWidth(int value);
  int getHeight();
  void setHeight(int value);
  org.jboss.netty.buffer.ChannelBuffer getData();
  void setData(org.jboss.netty.buffer.ChannelBuffer value);
}

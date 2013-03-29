package theora_image_transport;

public interface Packet extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "theora_image_transport/Packet";
  static final java.lang.String _DEFINITION = "# ROS message adaptation of the ogg_packet struct from libogg,\n# see http://www.xiph.org/ogg/doc/libogg/ogg_packet.html.\n\nHeader header     # Original sensor_msgs/Image header\nuint8[] data      # Raw Theora packet data (combines packet and bytes fields from ogg_packet)\nint32 b_o_s       # Flag indicating whether this packet begins a logical bitstream\nint32 e_o_s       # Flag indicating whether this packet ends a bitstream\nint64 granulepos  # A number indicating the position of this packet in the decoded data\nint64 packetno    # Sequential number of this packet in the ogg bitstream\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  org.jboss.netty.buffer.ChannelBuffer getData();
  void setData(org.jboss.netty.buffer.ChannelBuffer value);
  int getBOS();
  void setBOS(int value);
  int getEOS();
  void setEOS(int value);
  long getGranulepos();
  void setGranulepos(long value);
  long getPacketno();
  void setPacketno(long value);
}

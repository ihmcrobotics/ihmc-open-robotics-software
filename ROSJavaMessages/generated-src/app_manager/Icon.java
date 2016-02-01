package app_manager;

public interface Icon extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/Icon";
  static final java.lang.String _DEFINITION = "# Image data format.  \"jpeg\" or \"png\"\nstring format\n\n# Image data.\nuint8[] data\n";
  java.lang.String getFormat();
  void setFormat(java.lang.String value);
  org.jboss.netty.buffer.ChannelBuffer getData();
  void setData(org.jboss.netty.buffer.ChannelBuffer value);
}

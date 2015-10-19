package perf_roscpp;

public interface ThroughputMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "perf_roscpp/ThroughputMessage";
  static final java.lang.String _DEFINITION = "uint8[] array\n\n";
  org.jboss.netty.buffer.ChannelBuffer getArray();
  void setArray(org.jboss.netty.buffer.ChannelBuffer value);
}

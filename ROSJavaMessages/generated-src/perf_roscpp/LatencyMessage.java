package perf_roscpp;

public interface LatencyMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "perf_roscpp/LatencyMessage";
  static final java.lang.String _DEFINITION = "float64 publish_time\nfloat64 receipt_time\nuint64 count\nuint32 thread_index\nuint8[] array\n\n";
  double getPublishTime();
  void setPublishTime(double value);
  double getReceiptTime();
  void setReceiptTime(double value);
  long getCount();
  void setCount(long value);
  int getThreadIndex();
  void setThreadIndex(int value);
  org.jboss.netty.buffer.ChannelBuffer getArray();
  void setArray(org.jboss.netty.buffer.ChannelBuffer value);
}

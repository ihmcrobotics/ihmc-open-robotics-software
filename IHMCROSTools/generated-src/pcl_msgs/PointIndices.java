package pcl_msgs;

public interface PointIndices extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pcl_msgs/PointIndices";
  static final java.lang.String _DEFINITION = "Header header\nint32[] indices\n\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int[] getIndices();
  void setIndices(int[] value);
}

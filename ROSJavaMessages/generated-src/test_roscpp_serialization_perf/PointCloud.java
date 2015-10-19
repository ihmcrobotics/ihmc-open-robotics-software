package test_roscpp_serialization_perf;

public interface PointCloud extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization_perf/PointCloud";
  static final java.lang.String _DEFINITION = "Header header\nPoint32[] pts\nChannelFloat32[] chan\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<test_roscpp_serialization_perf.Point32> getPts();
  void setPts(java.util.List<test_roscpp_serialization_perf.Point32> value);
  java.util.List<test_roscpp_serialization_perf.ChannelFloat32> getChan();
  void setChan(java.util.List<test_roscpp_serialization_perf.ChannelFloat32> value);
}

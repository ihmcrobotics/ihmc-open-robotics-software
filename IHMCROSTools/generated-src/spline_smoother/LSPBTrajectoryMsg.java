package spline_smoother;

public interface LSPBTrajectoryMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "spline_smoother/LSPBTrajectoryMsg";
  static final java.lang.String _DEFINITION = "Header header\nstring[] names\nLSPBTrajectorySegmentMsg[] segments\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<java.lang.String> getNames();
  void setNames(java.util.List<java.lang.String> value);
  java.util.List<spline_smoother.LSPBTrajectorySegmentMsg> getSegments();
  void setSegments(java.util.List<spline_smoother.LSPBTrajectorySegmentMsg> value);
}

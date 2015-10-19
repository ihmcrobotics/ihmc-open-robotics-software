package spline_smoother;

public interface SplineTrajectory extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "spline_smoother/SplineTrajectory";
  static final java.lang.String _DEFINITION = "Header header\nstring[] names\nSplineTrajectorySegment[] segments\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<java.lang.String> getNames();
  void setNames(java.util.List<java.lang.String> value);
  java.util.List<spline_smoother.SplineTrajectorySegment> getSegments();
  void setSegments(java.util.List<spline_smoother.SplineTrajectorySegment> value);
}

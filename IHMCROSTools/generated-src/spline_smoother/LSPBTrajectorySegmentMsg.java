package spline_smoother;

public interface LSPBTrajectorySegmentMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "spline_smoother/LSPBTrajectorySegmentMsg";
  static final java.lang.String _DEFINITION = "duration duration\nLSPBSplineCoefficients[] joints\n";
  org.ros.message.Duration getDuration();
  void setDuration(org.ros.message.Duration value);
  java.util.List<spline_smoother.LSPBSplineCoefficients> getJoints();
  void setJoints(java.util.List<spline_smoother.LSPBSplineCoefficients> value);
}

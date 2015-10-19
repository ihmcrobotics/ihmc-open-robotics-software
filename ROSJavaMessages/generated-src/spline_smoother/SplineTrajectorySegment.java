package spline_smoother;

public interface SplineTrajectorySegment extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "spline_smoother/SplineTrajectorySegment";
  static final java.lang.String _DEFINITION = "duration duration\nSplineCoefficients[] joints\n";
  org.ros.message.Duration getDuration();
  void setDuration(org.ros.message.Duration value);
  java.util.List<spline_smoother.SplineCoefficients> getJoints();
  void setJoints(java.util.List<spline_smoother.SplineCoefficients> value);
}

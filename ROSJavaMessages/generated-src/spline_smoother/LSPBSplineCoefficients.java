package spline_smoother;

public interface LSPBSplineCoefficients extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "spline_smoother/LSPBSplineCoefficients";
  static final java.lang.String _DEFINITION = "float64[] coefficients\nfloat64 linear_segment_duration\nfloat64 quadratic_segment_duration";
  double[] getCoefficients();
  void setCoefficients(double[] value);
  double getLinearSegmentDuration();
  void setLinearSegmentDuration(double value);
  double getQuadraticSegmentDuration();
  void setQuadraticSegmentDuration(double value);
}

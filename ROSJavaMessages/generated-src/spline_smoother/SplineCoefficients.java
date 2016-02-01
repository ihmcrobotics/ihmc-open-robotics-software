package spline_smoother;

public interface SplineCoefficients extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "spline_smoother/SplineCoefficients";
  static final java.lang.String _DEFINITION = "float64[] coefficients\n";
  double[] getCoefficients();
  void setCoefficients(double[] value);
}

package atlas_msgs;

public interface GetJointDampingResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/GetJointDampingResponse";
  static final java.lang.String _DEFINITION = "# joint damping coefficient, and per joint bounds specified in AtlasPlugin.cc\nfloat64[28] damping_coefficients\nfloat64[28] damping_coefficients_min\nfloat64[28] damping_coefficients_max\nbool success\nstring status_message";
  double[] getDampingCoefficients();
  void setDampingCoefficients(double[] value);
  double[] getDampingCoefficientsMin();
  void setDampingCoefficientsMin(double[] value);
  double[] getDampingCoefficientsMax();
  void setDampingCoefficientsMax(double[] value);
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}

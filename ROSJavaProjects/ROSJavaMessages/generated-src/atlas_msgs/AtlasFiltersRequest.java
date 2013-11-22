package atlas_msgs;

public interface AtlasFiltersRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasFiltersRequest";
  static final java.lang.String _DEFINITION = "\nfloat64[] coef_a                      # filter coefficients.\n                                      # If set, must have length of 2,\n                                      # leave empty to keep previous values.\nfloat64[] coef_b                      # filter coefficients.\n                                      # If set, must have length of 2,\n                                      # leave empty to keep previous values.\n\nbool filter_velocity                  # turn velocity filter on or off\nbool filter_position                  # turn position filter on or off\n\n";
  double[] getCoefA();
  void setCoefA(double[] value);
  double[] getCoefB();
  void setCoefB(double[] value);
  boolean getFilterVelocity();
  void setFilterVelocity(boolean value);
  boolean getFilterPosition();
  void setFilterPosition(boolean value);
}

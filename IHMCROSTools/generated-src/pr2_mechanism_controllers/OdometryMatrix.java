package pr2_mechanism_controllers;

public interface OdometryMatrix extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_controllers/OdometryMatrix";
  static final java.lang.String _DEFINITION = "float64[] m";
  double[] getM();
  void setM(double[] value);
}

package pr2_mechanism_controllers;

public interface DebugInfo extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_controllers/DebugInfo";
  static final java.lang.String _DEFINITION = "float64[] timing\nint32 sequence\nbool input_valid\nfloat64 residual";
  double[] getTiming();
  void setTiming(double[] value);
  int getSequence();
  void setSequence(int value);
  boolean getInputValid();
  void setInputValid(boolean value);
  double getResidual();
  void setResidual(double value);
}

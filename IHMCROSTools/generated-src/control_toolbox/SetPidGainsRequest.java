package control_toolbox;

public interface SetPidGainsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "control_toolbox/SetPidGainsRequest";
  static final java.lang.String _DEFINITION = "float64 p\nfloat64 i\nfloat64 d\nfloat64 i_clamp\n";
  double getP();
  void setP(double value);
  double getI();
  void setI(double value);
  double getD();
  void setD(double value);
  double getIClamp();
  void setIClamp(double value);
}

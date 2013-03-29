package pr2_controllers_msgs;

public interface JointControllerState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_controllers_msgs/JointControllerState";
  static final java.lang.String _DEFINITION = "Header header\nfloat64 set_point\nfloat64 process_value\nfloat64 process_value_dot\nfloat64 error\nfloat64 time_step\nfloat64 command\nfloat64 p\nfloat64 i\nfloat64 d\nfloat64 i_clamp\n\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double getSetPoint();
  void setSetPoint(double value);
  double getProcessValue();
  void setProcessValue(double value);
  double getProcessValueDot();
  void setProcessValueDot(double value);
  double getError();
  void setError(double value);
  double getTimeStep();
  void setTimeStep(double value);
  double getCommand();
  void setCommand(double value);
  double getP();
  void setP(double value);
  double getI();
  void setI(double value);
  double getD();
  void setD(double value);
  double getIClamp();
  void setIClamp(double value);
}

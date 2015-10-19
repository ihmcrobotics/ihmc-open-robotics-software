package pr2_mechanism_controllers;

public interface SetProfileResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_controllers/SetProfileResponse";
  static final java.lang.String _DEFINITION = "float64 time";
  double getTime();
  void setTime(double value);
}

package pr2_mechanism_controllers;

public interface SetProfileRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_controllers/SetProfileRequest";
  static final java.lang.String _DEFINITION = "float64 UpperTurnaround\nfloat64 LowerTurnaround\nfloat64 upperDecelBuffer\nfloat64 lowerDecelBuffer\nfloat64 profile\nfloat64 period\nfloat64 amplitude\nfloat64 offset\n";
  double getUpperTurnaround();
  void setUpperTurnaround(double value);
  double getLowerTurnaround();
  void setLowerTurnaround(double value);
  double getUpperDecelBuffer();
  void setUpperDecelBuffer(double value);
  double getLowerDecelBuffer();
  void setLowerDecelBuffer(double value);
  double getProfile();
  void setProfile(double value);
  double getPeriod();
  void setPeriod(double value);
  double getAmplitude();
  void setAmplitude(double value);
  double getOffset();
  void setOffset(double value);
}

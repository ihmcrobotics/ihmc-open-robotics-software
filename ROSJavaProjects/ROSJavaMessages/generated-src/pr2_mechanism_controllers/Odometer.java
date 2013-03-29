package pr2_mechanism_controllers;

public interface Odometer extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_controllers/Odometer";
  static final java.lang.String _DEFINITION = "float64 distance #total distance traveled (meters)\nfloat64 angle #total angle traveled (radians)";
  double getDistance();
  void setDistance(double value);
  double getAngle();
  void setAngle(double value);
}

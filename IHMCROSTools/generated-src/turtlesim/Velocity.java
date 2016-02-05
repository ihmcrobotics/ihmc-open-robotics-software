package turtlesim;

public interface Velocity extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "turtlesim/Velocity";
  static final java.lang.String _DEFINITION = "float32 linear\nfloat32 angular";
  float getLinear();
  void setLinear(float value);
  float getAngular();
  void setAngular(float value);
}

package turtlesim;

public interface TeleportRelativeRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "turtlesim/TeleportRelativeRequest";
  static final java.lang.String _DEFINITION = "float32 linear\nfloat32 angular\n";
  float getLinear();
  void setLinear(float value);
  float getAngular();
  void setAngular(float value);
}

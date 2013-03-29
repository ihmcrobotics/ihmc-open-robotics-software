package turtlesim;

public interface TeleportAbsoluteRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "turtlesim/TeleportAbsoluteRequest";
  static final java.lang.String _DEFINITION = "float32 x\nfloat32 y\nfloat32 theta\n";
  float getX();
  void setX(float value);
  float getY();
  void setY(float value);
  float getTheta();
  void setTheta(float value);
}

package turtlesim;

public interface Pose extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "turtlesim/Pose";
  static final java.lang.String _DEFINITION = "float32 x\nfloat32 y\nfloat32 theta\n\nfloat32 linear_velocity\nfloat32 angular_velocity";
  float getX();
  void setX(float value);
  float getY();
  void setY(float value);
  float getTheta();
  void setTheta(float value);
  float getLinearVelocity();
  void setLinearVelocity(float value);
  float getAngularVelocity();
  void setAngularVelocity(float value);
}

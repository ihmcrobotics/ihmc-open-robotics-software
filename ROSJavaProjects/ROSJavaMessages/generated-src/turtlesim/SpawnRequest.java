package turtlesim;

public interface SpawnRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "turtlesim/SpawnRequest";
  static final java.lang.String _DEFINITION = "float32 x\nfloat32 y\nfloat32 theta\nstring name # Optional.  A unique name will be created and returned if this is empty\n";
  float getX();
  void setX(float value);
  float getY();
  void setY(float value);
  float getTheta();
  void setTheta(float value);
  java.lang.String getName();
  void setName(java.lang.String value);
}

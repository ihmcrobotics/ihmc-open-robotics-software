package handle_msgs;

public interface Collision extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "handle_msgs/Collision";
  static final java.lang.String _DEFINITION = "# This is basic collision message\n# it is used in HandleCollisions to build an array\n\nstring frame_id\n# finger[0]/proximal_link\n# finger[0]/distal_link\n# finger[1]/proximal_link\n# finger[1]/distal_link\n# finger[2]/proximal_link\n# finger[2]/distal_link\n# base_link\n\nint32 sensor_id\n# index of sensor\n\nfloat32 intensity\n\n# location of sensor on the surface of the finger in the link frame\nfloat32 x\nfloat32 y\nfloat32 z\n";
  java.lang.String getFrameId();
  void setFrameId(java.lang.String value);
  int getSensorId();
  void setSensorId(int value);
  float getIntensity();
  void setIntensity(float value);
  float getX();
  void setX(float value);
  float getY();
  void setY(float value);
  float getZ();
  void setZ(float value);
}

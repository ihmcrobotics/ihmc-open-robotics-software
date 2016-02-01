package multisense_ros;

public interface RawImuData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "multisense_ros/RawImuData";
  static final java.lang.String _DEFINITION = "time    time_stamp\nfloat32 x\nfloat32 y\nfloat32 z\n\n";
  org.ros.message.Time getTimeStamp();
  void setTimeStamp(org.ros.message.Time value);
  float getX();
  void setX(float value);
  float getY();
  void setY(float value);
  float getZ();
  void setZ(float value);
}

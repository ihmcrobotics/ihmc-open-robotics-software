package multisense_ros;

public interface RawLidarCal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "multisense_ros/RawLidarCal";
  static final java.lang.String _DEFINITION = "float32[16] laserToSpindle\nfloat32[16] cameraToSpindleFixed\n";
  float[] getLaserToSpindle();
  void setLaserToSpindle(float[] value);
  float[] getCameraToSpindleFixed();
  void setCameraToSpindleFixed(float[] value);
}

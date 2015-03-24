package polled_camera;

public interface GetPolledImageResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "polled_camera/GetPolledImageResponse";
  static final java.lang.String _DEFINITION = "# Timestamp of the captured image. Can be matched against incoming sensor_msgs/Image header.\ntime stamp";
  org.ros.message.Time getStamp();
  void setStamp(org.ros.message.Time value);
}

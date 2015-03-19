package polled_camera;

public interface GetPolledImageResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "polled_camera/GetPolledImageResponse";
  static final java.lang.String _DEFINITION = "bool success          # Could the image be captured?\nstring status_message # Error message in case of failure\ntime stamp            # Timestamp of the captured image. Can be matched\n                      # against incoming sensor_msgs/Image header.";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
  org.ros.message.Time getStamp();
  void setStamp(org.ros.message.Time value);
}

package sensor_msgs;

public interface SetCameraInfoRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_msgs/SetCameraInfoRequest";
  static final java.lang.String _DEFINITION = "# This service requests that a camera stores the given CameraInfo \n# as that camera\'s calibration information.\n#\n# The width and height in the camera_info field should match what the\n# camera is currently outputting on its camera_info topic, and the camera\n# will assume that the region of the imager that is being referred to is\n# the region that the camera is currently capturing.\n\nsensor_msgs/CameraInfo camera_info # The camera_info to store\n";
  sensor_msgs.CameraInfo getCameraInfo();
  void setCameraInfo(sensor_msgs.CameraInfo value);
}

package stereo_msgs;

public interface DisparityImage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "stereo_msgs/DisparityImage";
  static final java.lang.String _DEFINITION = "# Separate header for compatibility with current TimeSynchronizer.\n# Likely to be removed in a later release, use image.header instead.\nHeader header\n\n# Floating point disparity image. The disparities are pre-adjusted for any\n# x-offset between the principal points of the two cameras (in the case\n# that they are verged). That is: d = x_l - x_r - (cx_l - cx_r)\nsensor_msgs/Image image\n\n# Stereo geometry. For disparity d, the depth from the camera is Z = fT/d.\nfloat32 f # Focal length, pixels\nfloat32 T # Baseline, world units\n\n# Subwindow of (potentially) valid disparity values.\nsensor_msgs/RegionOfInterest valid_window\n\n# The range of disparities searched.\n# In the disparity image, any disparity less than min_disparity is invalid.\n# The disparity search range defines the horopter, or 3D volume that the\n# stereo algorithm can \"see\". Points with Z outside of:\n#     Z_min = fT / max_disparity\n#     Z_max = fT / min_disparity\n# could not be found.\nfloat32 min_disparity\nfloat32 max_disparity\n\n# Smallest allowed disparity increment. The smallest achievable depth range\n# resolution is delta_Z = (Z^2/fT)*delta_d.\nfloat32 delta_d\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  sensor_msgs.Image getImage();
  void setImage(sensor_msgs.Image value);
  float getF();
  void setF(float value);
  float getT();
  void setT(float value);
  sensor_msgs.RegionOfInterest getValidWindow();
  void setValidWindow(sensor_msgs.RegionOfInterest value);
  float getMinDisparity();
  void setMinDisparity(float value);
  float getMaxDisparity();
  void setMaxDisparity(float value);
  float getDeltaD();
  void setDeltaD(float value);
}

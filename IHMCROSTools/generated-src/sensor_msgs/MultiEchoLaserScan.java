package sensor_msgs;

public interface MultiEchoLaserScan extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_msgs/MultiEchoLaserScan";
  static final java.lang.String _DEFINITION = "# Single scan from a multi-echo planar laser range-finder\n#\n# If you have another ranging device with different behavior (e.g. a sonar\n# array), please find or create a different message, since applications\n# will make fairly laser-specific assumptions about this data\n\nHeader header            # timestamp in the header is the acquisition time of \n                         # the first ray in the scan.\n                         #\n                         # in frame frame_id, angles are measured around \n                         # the positive Z axis (counterclockwise, if Z is up)\n                         # with zero angle being forward along the x axis\n                         \nfloat32 angle_min        # start angle of the scan [rad]\nfloat32 angle_max        # end angle of the scan [rad]\nfloat32 angle_increment  # angular distance between measurements [rad]\n\nfloat32 time_increment   # time between measurements [seconds] - if your scanner\n                         # is moving, this will be used in interpolating position\n                         # of 3d points\nfloat32 scan_time        # time between scans [seconds]\n\nfloat32 range_min        # minimum range value [m]\nfloat32 range_max        # maximum range value [m]\n\nLaserEcho[] ranges       # range data [m] (Note: NaNs, values < range_min or > range_max should be discarded)\n                         # +Inf measurements are out of range\n                         # -Inf measurements are too close to determine exact distance.\nLaserEcho[] intensities  # intensity data [device-specific units].  If your\n                         # device does not provide intensities, please leave\n                         # the array empty.";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float getAngleMin();
  void setAngleMin(float value);
  float getAngleMax();
  void setAngleMax(float value);
  float getAngleIncrement();
  void setAngleIncrement(float value);
  float getTimeIncrement();
  void setTimeIncrement(float value);
  float getScanTime();
  void setScanTime(float value);
  float getRangeMin();
  void setRangeMin(float value);
  float getRangeMax();
  void setRangeMax(float value);
  java.util.List<sensor_msgs.LaserEcho> getRanges();
  void setRanges(java.util.List<sensor_msgs.LaserEcho> value);
  java.util.List<sensor_msgs.LaserEcho> getIntensities();
  void setIntensities(java.util.List<sensor_msgs.LaserEcho> value);
}

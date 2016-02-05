package sensor_msgs;

public interface Image extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_msgs/Image";
  static final java.lang.String _DEFINITION = "# This message contains an uncompressed image\n# (0, 0) is at top-left corner of image\n#\n\nHeader header        # Header timestamp should be acquisition time of image\n                     # Header frame_id should be optical frame of camera\n                     # origin of frame should be optical center of cameara\n                     # +x should point to the right in the image\n                     # +y should point down in the image\n                     # +z should point into to plane of the image\n                     # If the frame_id here and the frame_id of the CameraInfo\n                     # message associated with the image conflict\n                     # the behavior is undefined\n\nuint32 height         # image height, that is, number of rows\nuint32 width          # image width, that is, number of columns\n\n# The legal values for encoding are in file src/image_encodings.cpp\n# If you want to standardize a new string format, join\n# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\nstring encoding       # Encoding of pixels -- channel meaning, ordering, size\n                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n\nuint8 is_bigendian    # is this data bigendian?\nuint32 step           # Full row length in bytes\nuint8[] data          # actual matrix data, size is (step * rows)\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getHeight();
  void setHeight(int value);
  int getWidth();
  void setWidth(int value);
  java.lang.String getEncoding();
  void setEncoding(java.lang.String value);
  byte getIsBigendian();
  void setIsBigendian(byte value);
  int getStep();
  void setStep(int value);
  org.jboss.netty.buffer.ChannelBuffer getData();
  void setData(org.jboss.netty.buffer.ChannelBuffer value);
}

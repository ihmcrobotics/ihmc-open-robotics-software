package sensor_msgs;

public interface PointCloud2 extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_msgs/PointCloud2";
  static final java.lang.String _DEFINITION = "# This message holds a collection of N-dimensional points, which may\n# contain additional information such as normals, intensity, etc. The\n# point data is stored as a binary blob, its layout described by the\n# contents of the \"fields\" array.\n\n# The point cloud data may be organized 2d (image-like) or 1d\n# (unordered). Point clouds organized as 2d images may be produced by\n# camera depth sensors such as stereo or time-of-flight.\n\n# Time of sensor data acquisition, and the coordinate frame ID (for 3d\n# points).\nHeader header\n\n# 2D structure of the point cloud. If the cloud is unordered, height is\n# 1 and width is the length of the point cloud.\nuint32 height\nuint32 width\n\n# Describes the channels and their layout in the binary data blob.\nPointField[] fields\n\nbool    is_bigendian # Is this data bigendian?\nuint32  point_step   # Length of a point in bytes\nuint32  row_step     # Length of a row in bytes\nuint8[] data         # Actual point data, size is (row_step*height)\n\nbool is_dense        # True if there are no invalid points\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getHeight();
  void setHeight(int value);
  int getWidth();
  void setWidth(int value);
  java.util.List<sensor_msgs.PointField> getFields();
  void setFields(java.util.List<sensor_msgs.PointField> value);
  boolean getIsBigendian();
  void setIsBigendian(boolean value);
  int getPointStep();
  void setPointStep(int value);
  int getRowStep();
  void setRowStep(int value);
  org.jboss.netty.buffer.ChannelBuffer getData();
  void setData(org.jboss.netty.buffer.ChannelBuffer value);
  boolean getIsDense();
  void setIsDense(boolean value);
}

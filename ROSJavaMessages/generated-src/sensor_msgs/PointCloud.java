package sensor_msgs;

public interface PointCloud extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_msgs/PointCloud";
  static final java.lang.String _DEFINITION = "# This message holds a collection of 3d points, plus optional additional\n# information about each point.\n\n# Time of sensor data acquisition, coordinate frame ID.\nHeader header\n\n# Array of 3d points. Each Point32 should be interpreted as a 3d point\n# in the frame given in the header.\ngeometry_msgs/Point32[] points\n\n# Each channel should have the same number of elements as points array,\n# and the data in each channel should correspond 1:1 with each point.\n# Channel names in common practice are listed in ChannelFloat32.msg.\nChannelFloat32[] channels\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<geometry_msgs.Point32> getPoints();
  void setPoints(java.util.List<geometry_msgs.Point32> value);
  java.util.List<sensor_msgs.ChannelFloat32> getChannels();
  void setChannels(java.util.List<sensor_msgs.ChannelFloat32> value);
}

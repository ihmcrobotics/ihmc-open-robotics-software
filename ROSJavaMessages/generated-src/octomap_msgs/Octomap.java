package octomap_msgs;

public interface Octomap extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "octomap_msgs/Octomap";
  static final java.lang.String _DEFINITION = "# A 3D map in binary format, as Octree\nHeader header\n\n# Flag to denote a binary (only free/occupied) or full occupancy octree (.bt/.ot file)\nbool binary\n\n# Class id of the contained octree \nstring id\n\n# Resolution (in m) of the smallest octree nodes\nfloat64 resolution\n\n# binary serialization of octree, use conversions.h to read and write octrees\nint8[] data\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  boolean getBinary();
  void setBinary(boolean value);
  java.lang.String getId();
  void setId(java.lang.String value);
  double getResolution();
  void setResolution(double value);
  org.jboss.netty.buffer.ChannelBuffer getData();
  void setData(org.jboss.netty.buffer.ChannelBuffer value);
}

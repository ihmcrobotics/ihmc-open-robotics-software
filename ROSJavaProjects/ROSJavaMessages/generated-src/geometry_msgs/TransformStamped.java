package geometry_msgs;

public interface TransformStamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/TransformStamped";
  static final java.lang.String _DEFINITION = "# This expresses a transform from coordinate frame header.frame_id\n# to the coordinate frame child_frame_id\n#\n# This message is mostly used by the \n# <a href=\"http://www.ros.org/wiki/tf\">tf</a> package. \n# See its documentation for more information.\n\nHeader header\nstring child_frame_id # the frame id of the child frame\nTransform transform\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getChildFrameId();
  void setChildFrameId(java.lang.String value);
  geometry_msgs.Transform getTransform();
  void setTransform(geometry_msgs.Transform value);
}

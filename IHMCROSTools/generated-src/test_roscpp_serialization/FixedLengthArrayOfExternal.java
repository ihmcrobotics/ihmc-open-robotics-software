package test_roscpp_serialization;

public interface FixedLengthArrayOfExternal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp_serialization/FixedLengthArrayOfExternal";
  static final java.lang.String _DEFINITION = "# This comment has \"quotes\" in it and \\slashes\\\nrosgraph_msgs/Log[4] a";
  java.util.List<rosgraph_msgs.Log> getA();
  void setA(java.util.List<rosgraph_msgs.Log> value);
}

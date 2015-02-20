package smach_msgs;

public interface SmachContainerStatus extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "smach_msgs/SmachContainerStatus";
  static final java.lang.String _DEFINITION = "Header header\n\n# The path to this node in the server\nstring path\n\n# The initial state description\nstring[] initial_states\n\n# The current state description\nstring[] active_states\n\n# A pickled user data structure\nstring local_data\n\n# Debugging info string\nstring info\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getPath();
  void setPath(java.lang.String value);
  java.util.List<java.lang.String> getInitialStates();
  void setInitialStates(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getActiveStates();
  void setActiveStates(java.util.List<java.lang.String> value);
  java.lang.String getLocalData();
  void setLocalData(java.lang.String value);
  java.lang.String getInfo();
  void setInfo(java.lang.String value);
}

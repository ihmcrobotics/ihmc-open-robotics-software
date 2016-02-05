package smach_msgs;

public interface SmachContainerInitialStatusCmd extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "smach_msgs/SmachContainerInitialStatusCmd";
  static final java.lang.String _DEFINITION = "# The path to the node in the server\nstring path\n\n# The desired initial state(s)\nstring[] initial_states\n\n# Initial values for the local user data of the state machine\n# A pickled user data structure\n# i.e. the UserData\'s internal dictionary\nstring local_data\n";
  java.lang.String getPath();
  void setPath(java.lang.String value);
  java.util.List<java.lang.String> getInitialStates();
  void setInitialStates(java.util.List<java.lang.String> value);
  java.lang.String getLocalData();
  void setLocalData(java.lang.String value);
}

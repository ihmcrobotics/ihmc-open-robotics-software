package gazebo_msgs;

public interface ContactsState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/ContactsState";
  static final java.lang.String _DEFINITION = "Header header                                   # stamp\ngazebo_msgs/ContactState[] states            # array of geom pairs in contact\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<gazebo_msgs.ContactState> getStates();
  void setStates(java.util.List<gazebo_msgs.ContactState> value);
}

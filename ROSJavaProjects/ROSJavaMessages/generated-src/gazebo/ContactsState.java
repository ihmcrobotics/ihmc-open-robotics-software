package gazebo;

public interface ContactsState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/ContactsState";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nHeader header                                   # stamp\ngazebo/ContactState[] states            # array of geom pairs in contact\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<gazebo.ContactState> getStates();
  void setStates(java.util.List<gazebo.ContactState> value);
}

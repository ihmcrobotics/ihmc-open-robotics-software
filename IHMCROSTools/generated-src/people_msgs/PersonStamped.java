package people_msgs;

public interface PersonStamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "people_msgs/PersonStamped";
  static final java.lang.String _DEFINITION = "Header header\npeople_msgs/Person person\n\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  people_msgs.Person getPerson();
  void setPerson(people_msgs.Person value);
}

package people_msgs;

public interface People extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "people_msgs/People";
  static final java.lang.String _DEFINITION = "Header header\npeople_msgs/Person[] people\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<people_msgs.Person> getPeople();
  void setPeople(java.util.List<people_msgs.Person> value);
}

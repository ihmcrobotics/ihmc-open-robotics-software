package gazebo;

public interface BodyRequestRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/BodyRequestRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring body_name   # name of the body requested\n";
  java.lang.String getBodyName();
  void setBodyName(java.lang.String value);
}

package gazebo_msgs;

public interface BodyRequestRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/BodyRequestRequest";
  static final java.lang.String _DEFINITION = "string body_name   # name of the body requested. body names are prefixed by model name, e.g. pr2::base_link\n";
  java.lang.String getBodyName();
  void setBodyName(java.lang.String value);
}

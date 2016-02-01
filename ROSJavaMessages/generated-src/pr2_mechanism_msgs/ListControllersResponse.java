package pr2_mechanism_msgs;

public interface ListControllersResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_msgs/ListControllersResponse";
  static final java.lang.String _DEFINITION = "string[] controllers\nstring[] state";
  java.util.List<java.lang.String> getControllers();
  void setControllers(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getState();
  void setState(java.util.List<java.lang.String> value);
}

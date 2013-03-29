package pr2_mechanism_msgs;

public interface ListControllerTypesResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_msgs/ListControllerTypesResponse";
  static final java.lang.String _DEFINITION = "string[] types";
  java.util.List<java.lang.String> getTypes();
  void setTypes(java.util.List<java.lang.String> value);
}

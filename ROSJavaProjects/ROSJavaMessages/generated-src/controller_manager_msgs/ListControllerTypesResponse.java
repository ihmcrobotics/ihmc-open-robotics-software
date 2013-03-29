package controller_manager_msgs;

public interface ListControllerTypesResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "controller_manager_msgs/ListControllerTypesResponse";
  static final java.lang.String _DEFINITION = "string[] types\nstring[] base_classes";
  java.util.List<java.lang.String> getTypes();
  void setTypes(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getBaseClasses();
  void setBaseClasses(java.util.List<java.lang.String> value);
}

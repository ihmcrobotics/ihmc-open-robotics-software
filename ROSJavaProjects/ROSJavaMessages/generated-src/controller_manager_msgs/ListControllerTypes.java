package controller_manager_msgs;

public interface ListControllerTypes extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "controller_manager_msgs/ListControllerTypes";
  static final java.lang.String _DEFINITION = "# The ListControllers service returns a list of controller types that are known\n# to the controller manager plugin mechanism.\n\n---\nstring[] types\nstring[] base_classes\n";
}

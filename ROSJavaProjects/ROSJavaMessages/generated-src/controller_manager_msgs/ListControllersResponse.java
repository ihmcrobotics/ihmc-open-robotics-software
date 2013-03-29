package controller_manager_msgs;

public interface ListControllersResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "controller_manager_msgs/ListControllersResponse";
  static final java.lang.String _DEFINITION = "ControllerState[] controller";
  java.util.List<controller_manager_msgs.ControllerState> getController();
  void setController(java.util.List<controller_manager_msgs.ControllerState> value);
}

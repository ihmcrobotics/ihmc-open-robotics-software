package controller_manager_msgs;

public interface ControllersStatistics extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "controller_manager_msgs/ControllersStatistics";
  static final java.lang.String _DEFINITION = "std_msgs/Header header\ncontroller_manager_msgs/ControllerStatistics[] controller\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<controller_manager_msgs.ControllerStatistics> getController();
  void setController(java.util.List<controller_manager_msgs.ControllerStatistics> value);
}

package pr2_mechanism_msgs;

public interface UnloadControllerRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_msgs/UnloadControllerRequest";
  static final java.lang.String _DEFINITION = "# The UnloadController service allows you to unload a single controller \n# that is loaded in pr2_controller_manager. \n\n# To unload a controller, specify the \"name\" of the controller. \n# The return value \"ok\" indicates if the controller was unloaded or not.\n# There are three cases when unloadinng a controller will fail:\n#  * No controller with the specified name exists\n#  * The controller is still running\n#  * Another controller depends on the specified controller\n\nstring name\n";
  java.lang.String getName();
  void setName(java.lang.String value);
}

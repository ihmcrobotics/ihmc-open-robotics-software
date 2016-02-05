package controller_manager_msgs;

public interface UnloadController extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "controller_manager_msgs/UnloadController";
  static final java.lang.String _DEFINITION = "# The UnloadController service allows you to unload a single controller \n# from controller_manager\n\n# To unload a controller, specify the \"name\" of the controller. \n# The return value \"ok\" indicates if the controller was successfully\n# unloaded or not\n\nstring name\n---\nbool ok";
}

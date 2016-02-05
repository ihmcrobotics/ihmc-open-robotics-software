package controller_manager_msgs;

public interface LoadControllerRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "controller_manager_msgs/LoadControllerRequest";
  static final java.lang.String _DEFINITION = "# The LoadController service allows you to load a single controller \n# inside controller_manager\n\n# To load a controller, specify the \"name\" of the controller. \n# The return value \"ok\" indicates if the controller was successfully\n# constructed and initialized or not.\n\nstring name\n";
  java.lang.String getName();
  void setName(java.lang.String value);
}

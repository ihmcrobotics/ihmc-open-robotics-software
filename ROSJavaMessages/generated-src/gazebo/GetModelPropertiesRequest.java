package gazebo;

public interface GetModelPropertiesRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/GetModelPropertiesRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring model_name                    # name of Gazebo Model\n";
  java.lang.String getModelName();
  void setModelName(java.lang.String value);
}

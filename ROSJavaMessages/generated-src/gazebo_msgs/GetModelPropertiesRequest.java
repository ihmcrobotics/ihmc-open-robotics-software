package gazebo_msgs;

public interface GetModelPropertiesRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/GetModelPropertiesRequest";
  static final java.lang.String _DEFINITION = "string model_name                    # name of Gazebo Model\n";
  java.lang.String getModelName();
  void setModelName(java.lang.String value);
}

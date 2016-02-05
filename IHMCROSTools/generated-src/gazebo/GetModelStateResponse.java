package gazebo;

public interface GetModelStateResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/GetModelStateResponse";
  static final java.lang.String _DEFINITION = "geometry_msgs/Pose pose              # pose of model in relative entity frame\ngeometry_msgs/Twist twist            # twist of model in relative entity frame\nbool success                         # return true if get successful\nstring status_message                # comments if available";
  geometry_msgs.Pose getPose();
  void setPose(geometry_msgs.Pose value);
  geometry_msgs.Twist getTwist();
  void setTwist(geometry_msgs.Twist value);
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}

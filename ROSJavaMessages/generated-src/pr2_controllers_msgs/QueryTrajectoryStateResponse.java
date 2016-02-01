package pr2_controllers_msgs;

public interface QueryTrajectoryStateResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_controllers_msgs/QueryTrajectoryStateResponse";
  static final java.lang.String _DEFINITION = "string[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] acceleration";
  java.util.List<java.lang.String> getName();
  void setName(java.util.List<java.lang.String> value);
  double[] getPosition();
  void setPosition(double[] value);
  double[] getVelocity();
  void setVelocity(double[] value);
  double[] getAcceleration();
  void setAcceleration(double[] value);
}

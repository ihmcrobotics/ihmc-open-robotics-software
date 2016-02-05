package robot_pose_ekf;

public interface GetStatusResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_pose_ekf/GetStatusResponse";
  static final java.lang.String _DEFINITION = "string status";
  java.lang.String getStatus();
  void setStatus(java.lang.String value);
}

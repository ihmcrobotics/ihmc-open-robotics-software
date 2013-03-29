package trajectory_msgs;

public interface JointTrajectory extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "trajectory_msgs/JointTrajectory";
  static final java.lang.String _DEFINITION = "Header header\nstring[] joint_names\nJointTrajectoryPoint[] points";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<java.lang.String> getJointNames();
  void setJointNames(java.util.List<java.lang.String> value);
  java.util.List<trajectory_msgs.JointTrajectoryPoint> getPoints();
  void setPoints(java.util.List<trajectory_msgs.JointTrajectoryPoint> value);
}

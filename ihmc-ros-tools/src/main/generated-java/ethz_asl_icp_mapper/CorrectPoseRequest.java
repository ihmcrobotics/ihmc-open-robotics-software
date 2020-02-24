package ethz_asl_icp_mapper;

public interface CorrectPoseRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethz_asl_icp_mapper/CorrectPoseRequest";
  static final java.lang.String _DEFINITION = "nav_msgs/Odometry odom\n";
  nav_msgs.Odometry getOdom();
  void setOdom(nav_msgs.Odometry value);
}

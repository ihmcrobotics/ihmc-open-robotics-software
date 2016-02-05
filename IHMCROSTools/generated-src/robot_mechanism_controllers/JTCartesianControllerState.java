package robot_mechanism_controllers;

public interface JTCartesianControllerState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_mechanism_controllers/JTCartesianControllerState";
  static final java.lang.String _DEFINITION = "Header header\ngeometry_msgs/PoseStamped x\ngeometry_msgs/PoseStamped x_desi\ngeometry_msgs/PoseStamped x_desi_filtered\ngeometry_msgs/Twist x_err\ngeometry_msgs/Twist xd\ngeometry_msgs/Twist xd_desi\ngeometry_msgs/Wrench F\nfloat64[] tau_pose\nfloat64[] tau_posture\nfloat64[] tau\nstd_msgs/Float64MultiArray J\nstd_msgs/Float64MultiArray N\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.PoseStamped getX();
  void setX(geometry_msgs.PoseStamped value);
  geometry_msgs.PoseStamped getXDesi();
  void setXDesi(geometry_msgs.PoseStamped value);
  geometry_msgs.PoseStamped getXDesiFiltered();
  void setXDesiFiltered(geometry_msgs.PoseStamped value);
  geometry_msgs.Twist getXErr();
  void setXErr(geometry_msgs.Twist value);
  geometry_msgs.Twist getXd();
  void setXd(geometry_msgs.Twist value);
  geometry_msgs.Twist getXdDesi();
  void setXdDesi(geometry_msgs.Twist value);
  geometry_msgs.Wrench getF();
  void setF(geometry_msgs.Wrench value);
  double[] getTauPose();
  void setTauPose(double[] value);
  double[] getTauPosture();
  void setTauPosture(double[] value);
  double[] getTau();
  void setTau(double[] value);
  std_msgs.Float64MultiArray getJ();
  void setJ(std_msgs.Float64MultiArray value);
  std_msgs.Float64MultiArray getN();
  void setN(std_msgs.Float64MultiArray value);
}

package geometry_msgs;

public interface PoseWithCovariance extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/PoseWithCovariance";
  static final java.lang.String _DEFINITION = "# This represents a pose in free space with uncertainty.\n\nPose pose\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n";
  geometry_msgs.Pose getPose();
  void setPose(geometry_msgs.Pose value);
  double[] getCovariance();
  void setCovariance(double[] value);
}

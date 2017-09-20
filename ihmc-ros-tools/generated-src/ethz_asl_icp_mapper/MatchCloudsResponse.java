package ethz_asl_icp_mapper;

public interface MatchCloudsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethz_asl_icp_mapper/MatchCloudsResponse";
  static final java.lang.String _DEFINITION = "geometry_msgs/Transform transform";
  geometry_msgs.Transform getTransform();
  void setTransform(geometry_msgs.Transform value);
}

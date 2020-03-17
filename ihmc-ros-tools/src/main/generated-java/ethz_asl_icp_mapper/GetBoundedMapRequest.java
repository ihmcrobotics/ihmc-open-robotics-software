package ethz_asl_icp_mapper;

public interface GetBoundedMapRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethz_asl_icp_mapper/GetBoundedMapRequest";
  static final java.lang.String _DEFINITION = "geometry_msgs/Pose mapCenter\ngeometry_msgs/Point topRightCorner\ngeometry_msgs/Point bottomLeftCorner\n";
  geometry_msgs.Pose getMapCenter();
  void setMapCenter(geometry_msgs.Pose value);
  geometry_msgs.Point getTopRightCorner();
  void setTopRightCorner(geometry_msgs.Point value);
  geometry_msgs.Point getBottomLeftCorner();
  void setBottomLeftCorner(geometry_msgs.Point value);
}

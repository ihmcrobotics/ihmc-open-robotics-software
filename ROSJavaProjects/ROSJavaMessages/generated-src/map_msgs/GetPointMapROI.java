package map_msgs;

public interface GetPointMapROI extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_msgs/GetPointMapROI";
  static final java.lang.String _DEFINITION = "float64 x\nfloat64 y\nfloat64 z\nfloat64 r    # if != 0, circular ROI of radius r\nfloat64 l_x  # if r == 0, length of AABB on x\nfloat64 l_y  # if r == 0, length of AABB on y\nfloat64 l_z  # if r == 0, length of AABB on z\n---\nsensor_msgs/PointCloud2 sub_map";
}

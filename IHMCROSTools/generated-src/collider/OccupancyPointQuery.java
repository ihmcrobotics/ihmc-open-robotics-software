package collider;

public interface OccupancyPointQuery extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "collider/OccupancyPointQuery";
  static final java.lang.String _DEFINITION = "geometry_msgs/Point point\n---\nint8 occupancy\nint8 FREE=0\nint8 OCCUPIED=1\nint8 UNKNOWN=-1";
}

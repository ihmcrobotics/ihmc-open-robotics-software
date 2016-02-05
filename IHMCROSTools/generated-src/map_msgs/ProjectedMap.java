package map_msgs;

public interface ProjectedMap extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_msgs/ProjectedMap";
  static final java.lang.String _DEFINITION = "nav_msgs/OccupancyGrid map\nfloat64 min_z\nfloat64 max_z";
  nav_msgs.OccupancyGrid getMap();
  void setMap(nav_msgs.OccupancyGrid value);
  double getMinZ();
  void setMinZ(double value);
  double getMaxZ();
  void setMaxZ(double value);
}

package map_msgs;

public interface GetMapROIResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_msgs/GetMapROIResponse";
  static final java.lang.String _DEFINITION = "nav_msgs/OccupancyGrid sub_map";
  nav_msgs.OccupancyGrid getSubMap();
  void setSubMap(nav_msgs.OccupancyGrid value);
}

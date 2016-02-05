package nav_msgs;

public interface GetMapResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "nav_msgs/GetMapResponse";
  static final java.lang.String _DEFINITION = "nav_msgs/OccupancyGrid map";
  nav_msgs.OccupancyGrid getMap();
  void setMap(nav_msgs.OccupancyGrid value);
}

package grid_map_msg;

public interface GetGridMapResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "grid_map_msg/GetGridMapResponse";
  static final java.lang.String _DEFINITION = "\n# Submap\ngrid_map_msg/GridMap gridMap";
  grid_map_msg.GridMap getGridMap();
  void setGridMap(grid_map_msg.GridMap value);
}

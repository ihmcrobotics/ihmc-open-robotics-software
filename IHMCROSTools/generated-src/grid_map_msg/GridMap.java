package grid_map_msg;

public interface GridMap extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "grid_map_msg/GridMap";
  static final java.lang.String _DEFINITION = "# Grid map header\nGridMapInfo info\n\n# Grid map data definition.\nstd_msgs/String[] dataDefinition\n\n# Grid map data.\nstd_msgs/Float32MultiArray[] data\n\n# Row start index (default 0).\nuint16 outerStartIndex\n\n# Column start index (default 0).\nuint16 innerStartIndex";
  grid_map_msg.GridMapInfo getInfo();
  void setInfo(grid_map_msg.GridMapInfo value);
  java.util.List<std_msgs.String> getDataDefinition();
  void setDataDefinition(java.util.List<std_msgs.String> value);
  java.util.List<std_msgs.Float32MultiArray> getData();
  void setData(java.util.List<std_msgs.Float32MultiArray> value);
  short getOuterStartIndex();
  void setOuterStartIndex(short value);
  short getInnerStartIndex();
  void setInnerStartIndex(short value);
}

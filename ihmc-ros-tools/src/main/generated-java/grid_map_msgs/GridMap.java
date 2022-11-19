package grid_map_msgs;

import std_msgs.Float32MultiArray;
import us.ihmc.idl.IDLSequence;

public interface GridMap extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "grid_map_msgs/GridMap";
  static final java.lang.String _DEFINITION = "# Grid map header\nGridMapInfo info\n\n# Grid map data definition.\nstd_msgs/String[] dataDefinition\n\n# Grid map data.\nstd_msgs/Float32MultiArray[] data\n\n# Row start index (default 0).\nuint16 outerStartIndex\n\n# Column start index (default 0).\nuint16 innerStartIndex";
  grid_map_msgs.GridMapInfo getInfo();
  void setInfo(grid_map_msgs.GridMapInfo value);
  String[] getLayers();
  void setLayers (String[] value);
  String[] getBasicLayers();
  void setBasicLayers(String[] value);
  std_msgs.Float32MultiArray[] getData();
  void setData(std_msgs.Float32MultiArray[] value);
  int getOuterStartIndex();
  void setOuterStartIndex(int value);
  int getInnerStartIndex();
  void setInnerStartIndex(int value);
}

package grid_map_msg;

public interface GetGridMap extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "grid_map_msg/GetGridMap";
  static final java.lang.String _DEFINITION = "# Requested submap position in x-direction [m].\nfloat64 positionX\n  \n# Requested submap position in y-direction [m].\nfloat64 positionY\n\n# Requested submap length in x-direction [m].\nfloat64 lengthX\n  \n# Requested submap width in y-direction [m].\nfloat64 lengthY\n\n# Requested data. If empty, get all available data.\nstd_msgs/String[] dataDefinition\n\n---\n\n# Submap\ngrid_map_msg/GridMap gridMap";
}

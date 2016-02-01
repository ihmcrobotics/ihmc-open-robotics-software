package costmap_2d;

public interface VoxelGrid extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "costmap_2d/VoxelGrid";
  static final java.lang.String _DEFINITION = "Header header\nuint32[] data\ngeometry_msgs/Point32 origin\ngeometry_msgs/Vector3 resolutions\nuint32 size_x\nuint32 size_y\nuint32 size_z\n\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int[] getData();
  void setData(int[] value);
  geometry_msgs.Point32 getOrigin();
  void setOrigin(geometry_msgs.Point32 value);
  geometry_msgs.Vector3 getResolutions();
  void setResolutions(geometry_msgs.Vector3 value);
  int getSizeX();
  void setSizeX(int value);
  int getSizeY();
  void setSizeY(int value);
  int getSizeZ();
  void setSizeZ(int value);
}

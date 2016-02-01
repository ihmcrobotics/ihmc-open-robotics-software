package nav_msgs;

public interface GridCells extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "nav_msgs/GridCells";
  static final java.lang.String _DEFINITION = "#an array of cells in a 2D grid\nHeader header\nfloat32 cell_width\nfloat32 cell_height\ngeometry_msgs/Point[] cells\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float getCellWidth();
  void setCellWidth(float value);
  float getCellHeight();
  void setCellHeight(float value);
  java.util.List<geometry_msgs.Point> getCells();
  void setCells(java.util.List<geometry_msgs.Point> value);
}

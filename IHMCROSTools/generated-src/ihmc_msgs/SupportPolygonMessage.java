package ihmc_msgs;

public interface SupportPolygonMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/SupportPolygonMessage";
  static final java.lang.String _DEFINITION = "## SupportPolygonMessage\n# This message contains of an array of points that represent the\n# convex hull of a support polygon for a single robot foot\n\n# Constant defining max number of possible elements in the array of points\nint32 MAXIMUM_NUMBER_OF_VERTICES=8\n\n# The number of vertices in the array of points\nint32 number_of_vertices\n\n# The vertices of the support polygon\nihmc_msgs/Point2dMessage[] vertices";
  static final int MAXIMUM_NUMBER_OF_VERTICES = 8;
  int getNumberOfVertices();
  void setNumberOfVertices(int value);
  java.util.List<ihmc_msgs.Point2dMessage> getVertices();
  void setVertices(java.util.List<ihmc_msgs.Point2dMessage> value);
}

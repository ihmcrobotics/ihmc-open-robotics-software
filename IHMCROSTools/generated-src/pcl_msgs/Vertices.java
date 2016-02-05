package pcl_msgs;

public interface Vertices extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pcl_msgs/Vertices";
  static final java.lang.String _DEFINITION = "# List of point indices\nuint32[] vertices\n";
  int[] getVertices();
  void setVertices(int[] value);
}

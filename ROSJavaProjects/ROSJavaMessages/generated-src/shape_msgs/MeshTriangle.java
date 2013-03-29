package shape_msgs;

public interface MeshTriangle extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "shape_msgs/MeshTriangle";
  static final java.lang.String _DEFINITION = "# Definition of a triangle\'s vertices\nuint32[3] vertex_indices\n";
  int[] getVertexIndices();
  void setVertexIndices(int[] value);
}

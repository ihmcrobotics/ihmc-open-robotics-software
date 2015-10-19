package shape_msgs;

public interface Mesh extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "shape_msgs/Mesh";
  static final java.lang.String _DEFINITION = "# Definition of a mesh\n\n# list of triangles; the index values refer to positions in vertices[]\nMeshTriangle[] triangles\n\n# the actual vertices that make up the mesh\ngeometry_msgs/Point[] vertices\n";
  java.util.List<shape_msgs.MeshTriangle> getTriangles();
  void setTriangles(java.util.List<shape_msgs.MeshTriangle> value);
  java.util.List<geometry_msgs.Point> getVertices();
  void setVertices(java.util.List<geometry_msgs.Point> value);
}

package arm_navigation_msgs;

public interface Shape extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/Shape";
  static final java.lang.String _DEFINITION = "byte SPHERE=0\nbyte BOX=1\nbyte CYLINDER=2\nbyte MESH=3\n\nbyte type\n\n\n#### define sphere, box, cylinder ####\n# the origin of each shape is considered at the shape\'s center\n\n# for sphere\n# radius := dimensions[0]\n\n# for cylinder\n# radius := dimensions[0]\n# length := dimensions[1]\n# the length is along the Z axis\n\n# for box\n# size_x := dimensions[0]\n# size_y := dimensions[1]\n# size_z := dimensions[2]\nfloat64[] dimensions\n\n\n#### define mesh ####\n\n# list of triangles; triangle k is defined by tre vertices located\n# at indices triangles[3k], triangles[3k+1], triangles[3k+2]\nint32[] triangles\ngeometry_msgs/Point[] vertices\n";
  static final byte SPHERE = 0;
  static final byte BOX = 1;
  static final byte CYLINDER = 2;
  static final byte MESH = 3;
  byte getType();
  void setType(byte value);
  double[] getDimensions();
  void setDimensions(double[] value);
  int[] getTriangles();
  void setTriangles(int[] value);
  java.util.List<geometry_msgs.Point> getVertices();
  void setVertices(java.util.List<geometry_msgs.Point> value);
}

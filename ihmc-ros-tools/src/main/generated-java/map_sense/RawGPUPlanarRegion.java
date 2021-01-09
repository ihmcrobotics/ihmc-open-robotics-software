package map_sense;

public interface RawGPUPlanarRegion extends org.ros.internal.message.Message {
  static final String _TYPE = "map_sense/RawGPUPlanarRegion";
  static final String _DEFINITION = "uint16 id\n" +
                                    "uint16 numOfPatches\n" +
                                    "geometry_msgs/Vector3 normal\n" +
                                    "geometry_msgs/Point centroid\n" +
                                    "geometry_msgs/Point[] vertices";
  std_msgs.UInt16 getId();
  void setId(std_msgs.UInt16 id);
  std_msgs.UInt16 getNumOfPatches();
  void setNumOfPatches(std_msgs.UInt16 numOfPatches);
  geometry_msgs.Point getCentroid();
  void setCentroid(geometry_msgs.Point value);
  geometry_msgs.Vector3 getNormal();
  void setNormal(geometry_msgs.Vector3 value);
  geometry_msgs.Point[] getVertices();
  void setVertices(geometry_msgs.Point[] value);
}

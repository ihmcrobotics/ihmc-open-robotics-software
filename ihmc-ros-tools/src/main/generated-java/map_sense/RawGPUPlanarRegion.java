package map_sense;

import geometry_msgs.Point;

import java.util.List;

public interface RawGPUPlanarRegion extends org.ros.internal.message.Message
{
   static final String _TYPE = "map_sense/RawGPUPlanarRegion";
   static final String _DEFINITION =
         "uint16 id\n" + "uint16 numOfPatches\n" + "geometry_msgs/Vector3 normal\n" + "geometry_msgs/Point centroid\n" + "geometry_msgs/Point[] vertices";

   Short getId();

   void setId(Short id);

   Short getNumOfPatches();

   void setNumOfPatches(Short numOfPatches);

   geometry_msgs.Point getCentroid();

   void setCentroid(geometry_msgs.Point value);

   geometry_msgs.Vector3 getNormal();

   void setNormal(geometry_msgs.Vector3 value);

   List<Point> getVertices();

   void setVertices(List<geometry_msgs.Point> value);
}

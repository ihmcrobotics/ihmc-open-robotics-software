package us.ihmc.communication.packets;

import java.util.List;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import us.ihmc.robotics.geometry.PlanarRegion;

public class PlanarRegionMessage extends Packet<PlanarRegionMessage>
{
   public int regionId = PlanarRegion.NO_REGION_ID;
   public Point3f regionOrigin;
   public Vector3f regionNormal;
   public Point2f[] concaveHullVertices;
   public List<Point2f[]> convexPolygonsVertices;

   public PlanarRegionMessage()
   {
   }

   public PlanarRegionMessage(Point3f regionOrigin, Vector3f regionNormal, Point2f[] concaveHullVertices, List<Point2f[]> convexPolygonsVertices)
   {
      this.regionOrigin = regionOrigin;
      this.regionNormal = regionNormal;
      this.concaveHullVertices = concaveHullVertices;
      this.convexPolygonsVertices = convexPolygonsVertices;
   }

   public void setRegionId(int regionId)
   {
      this.regionId = regionId;
   }

   public int getRegionId()
   {
      return regionId;
   }

   public Point3f getRegionOrigin()
   {
      return regionOrigin;
   }

   public Vector3f getRegionNormal()
   {
      return regionNormal;
   }

   public List<Point2f[]> getConvexPolygonsVertices()
   {
      return convexPolygonsVertices;
   }

   @Override
   public boolean epsilonEquals(PlanarRegionMessage other, double epsilon)
   {
      if (regionId != other.regionId)
         return false;
      if (!regionOrigin.epsilonEquals(other.regionOrigin, (float) epsilon))
         return false;
      if (!regionNormal.epsilonEquals(other.regionNormal, (float) epsilon))
         return false;
      if (convexPolygonsVertices.size() != other.convexPolygonsVertices.size())
         return false;
      if (concaveHullVertices.length != other.concaveHullVertices.length)
         return false;

      for (int vertexIndex = 0; vertexIndex < concaveHullVertices.length; vertexIndex++)
      {
         Point2f thisVertex = concaveHullVertices[vertexIndex];
         Point2f otherVertex = other.concaveHullVertices[vertexIndex];
         if (!thisVertex.epsilonEquals(otherVertex, (float) epsilon))
            return false;
      }

      for (int polygonIndex = 0; polygonIndex < convexPolygonsVertices.size(); polygonIndex++)
      {
         Point2f[] thisPolygon = convexPolygonsVertices.get(polygonIndex);
         Point2f[] otherPolygon = other.convexPolygonsVertices.get(polygonIndex);

         if (thisPolygon.length != otherPolygon.length)
            return false;

         for (int vertexIndex = 0; vertexIndex < thisPolygon.length; vertexIndex++)
         {
            if (!thisPolygon[vertexIndex].epsilonEquals(otherPolygon[polygonIndex], (float) epsilon))
               return false;
         }
      }
      return true;
   }
}

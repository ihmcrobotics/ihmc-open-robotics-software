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
   public List<Point2f[]> concaveHullsVertices;
   public List<Point2f[]> convexPolygonsVertices;

   public PlanarRegionMessage()
   {
   }

   public PlanarRegionMessage(Point3f regionOrigin, Vector3f regionNormal, List<Point2f[]> concaveHullsVertices, List<Point2f[]> convexPolygonsVertices)
   {
      this.regionOrigin = regionOrigin;
      this.regionNormal = regionNormal;
      this.concaveHullsVertices = concaveHullsVertices;
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

   public List<Point2f[]> getConcaveHullsVertices()
   {
      return concaveHullsVertices;
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
      if (concaveHullsVertices.size() != other.concaveHullsVertices.size())
         return false;

      for (int hullIndex = 0; hullIndex < concaveHullsVertices.size(); hullIndex++)
      {
         Point2f[] thisHull = concaveHullsVertices.get(hullIndex);
         Point2f[] otherHull = other.concaveHullsVertices.get(hullIndex);

         if (thisHull.length != otherHull.length)
            return false;

         for (int vertexIndex = 0; vertexIndex < thisHull.length; vertexIndex++)
         {
            Point2f thisVertex = thisHull[vertexIndex];
            Point2f otherVertex = otherHull[vertexIndex];
            if (!thisVertex.epsilonEquals(otherVertex, (float) epsilon))
               return false;
         }
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

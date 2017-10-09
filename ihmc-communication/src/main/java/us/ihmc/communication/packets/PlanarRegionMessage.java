package us.ihmc.communication.packets;

import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.robotics.geometry.PlanarRegion;

public class PlanarRegionMessage extends Packet<PlanarRegionMessage>
{
   public int regionId = PlanarRegion.NO_REGION_ID;
   public Point3D32 regionOrigin;
   public Vector3D32 regionNormal;
   public List<Point2D32[]> concaveHullsVertices;
   public List<Point2D32[]> convexPolygonsVertices;

   public PlanarRegionMessage()
   {
   }

   public PlanarRegionMessage(Point3D32 regionOrigin, Vector3D32 regionNormal, List<Point2D32[]> concaveHullsVertices, List<Point2D32[]> convexPolygonsVertices)
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

   public Point3D32 getRegionOrigin()
   {
      return regionOrigin;
   }

   public Vector3D32 getRegionNormal()
   {
      return regionNormal;
   }

   public List<Point2D32[]> getConcaveHullsVertices()
   {
      return concaveHullsVertices;
   }

   public List<Point2D32[]> getConvexPolygonsVertices()
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
         Point2D32[] thisHull = concaveHullsVertices.get(hullIndex);
         Point2D32[] otherHull = other.concaveHullsVertices.get(hullIndex);

         if (thisHull.length != otherHull.length)
            return false;

         for (int vertexIndex = 0; vertexIndex < thisHull.length; vertexIndex++)
         {
            Point2D32 thisVertex = thisHull[vertexIndex];
            Point2D32 otherVertex = otherHull[vertexIndex];
            if (!thisVertex.epsilonEquals(otherVertex, (float) epsilon))
               return false;
         }
      }

      for (int polygonIndex = 0; polygonIndex < convexPolygonsVertices.size(); polygonIndex++)
      {
         Point2D32[] thisPolygon = convexPolygonsVertices.get(polygonIndex);
         Point2D32[] otherPolygon = other.convexPolygonsVertices.get(polygonIndex);

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

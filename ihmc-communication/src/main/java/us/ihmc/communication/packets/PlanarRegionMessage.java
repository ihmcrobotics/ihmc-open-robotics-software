package us.ihmc.communication.packets;

import java.util.ArrayList;
import java.util.Arrays;
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
   public Point2D32[] concaveHullVertices;
   public List<Point2D32[]> convexPolygonsVertices;

   public PlanarRegionMessage()
   {
   }

   public PlanarRegionMessage(Point3D32 regionOrigin, Vector3D32 regionNormal, Point2D32[] concaveHullVertices, List<Point2D32[]> convexPolygonsVertices)
   {
      this.regionOrigin = regionOrigin;
      this.regionNormal = regionNormal;
      this.concaveHullVertices = concaveHullVertices;
      this.convexPolygonsVertices = convexPolygonsVertices;
   }

   @Override
   public void set(PlanarRegionMessage other)
   {
      regionId = other.regionId;
      regionOrigin = new Point3D32(other.regionOrigin);
      regionNormal = new Vector3D32(other.regionNormal);
      concaveHullVertices = Arrays.stream(other.concaveHullVertices).map(Point2D32::new).toArray(Point2D32[]::new);
      convexPolygonsVertices = new ArrayList<>(other.convexPolygonsVertices.size());
      for (int i = 0; i < other.convexPolygonsVertices.size(); i++)
      {
         Point2D32[] vertices = Arrays.stream(other.convexPolygonsVertices.get(i)).map(Point2D32::new).toArray(Point2D32[]::new);
         convexPolygonsVertices.add(vertices);
      }
      setPacketInformation(other);
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

   public int getConcaveHullSize()
   {
      return concaveHullVertices.length;
   }

   public Point2D32[] getConcaveHullVertices()
   {
      return concaveHullVertices;
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
      if (concaveHullVertices.length != other.concaveHullVertices.length)
         return false;

      for (int vertexIndex = 0; vertexIndex < concaveHullVertices.length; vertexIndex++)
      {
         Point2D32 thisVertex = concaveHullVertices[vertexIndex];
         Point2D32 otherVertex = other.concaveHullVertices[vertexIndex];
         if (!thisVertex.epsilonEquals(otherVertex, (float) epsilon))
            return false;
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

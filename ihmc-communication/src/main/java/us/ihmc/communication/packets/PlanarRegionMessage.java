package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.idl.RecyclingArrayListPubSub;
import us.ihmc.robotics.geometry.PlanarRegion;

public class PlanarRegionMessage extends Packet<PlanarRegionMessage>
{
   public int regionId = PlanarRegion.NO_REGION_ID;
   public Point3D32 regionOrigin;
   public Vector3D32 regionNormal;
   public Polygon2DMessage concaveHull = new Polygon2DMessage();
   public RecyclingArrayListPubSub<Polygon2DMessage> convexPolygons = new RecyclingArrayListPubSub<>(Polygon2DMessage.class, Polygon2DMessage::new, 5);

   public PlanarRegionMessage()
   {
   }

   @Override
   public void set(PlanarRegionMessage other)
   {
      regionId = other.regionId;
      regionOrigin = new Point3D32(other.regionOrigin);
      regionNormal = new Vector3D32(other.regionNormal);
      concaveHull.set(other.concaveHull);
      MessageTools.copyData(other.convexPolygons, convexPolygons);
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
      return concaveHull.getVertices().size();
   }

   public Polygon2DMessage getConcaveHull()
   {
      return concaveHull;
   }

   public RecyclingArrayListPubSub<Polygon2DMessage> getConvexPolygons()
   {
      return convexPolygons;
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
      if (!concaveHull.epsilonEquals(other.concaveHull, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(convexPolygons, other.convexPolygons, epsilon))
         return false;

      return true;
   }
}

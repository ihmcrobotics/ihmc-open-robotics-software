package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.RecyclingArrayListPubSub;

public class DetectedFacesPacket extends Packet<DetectedFacesPacket>
{
   public RecyclingArrayListPubSub<StringBuilder> ids = new RecyclingArrayListPubSub<>(StringBuilder.class, StringBuilder::new, 10);
   public RecyclingArrayListPubSub<Point3D> positions = new RecyclingArrayListPubSub<>(Point3D.class, Point3D::new, 10);

   public DetectedFacesPacket()
   {
   }

   @Override
   public void set(DetectedFacesPacket other)
   {
      MessageTools.copyData(other.ids, ids);
      MessageTools.copyData(other.positions, positions);
      setPacketInformation(other);
   }

   public RecyclingArrayListPubSub<StringBuilder> getIds()
   {
      return ids;
   }

   public RecyclingArrayListPubSub<Point3D> getPositions()
   {
      return positions;
   }

   @Override
   public boolean epsilonEquals(DetectedFacesPacket other, double epsilon)
   {
      if (!ids.equals(other.ids))
         return false;
      if (!MessageTools.epsilonEquals(positions, other.positions, epsilon))
         return false;
      return true;
   }
}

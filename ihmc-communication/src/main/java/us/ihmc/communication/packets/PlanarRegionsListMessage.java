package us.ihmc.communication.packets;

import us.ihmc.idl.RecyclingArrayListPubSub;

// FIXME Refactor to hold onto a single vertex buffer.
public class PlanarRegionsListMessage extends Packet<PlanarRegionsListMessage>
{
   public RecyclingArrayListPubSub<PlanarRegionMessage> planarRegions = new RecyclingArrayListPubSub<>(PlanarRegionMessage.class, PlanarRegionMessage::new, 500);

   public PlanarRegionsListMessage()
   {
   }

   @Override
   public void set(PlanarRegionsListMessage other)
   {
      MessageTools.copyData(other.planarRegions, planarRegions);
      setPacketInformation(other);
   }

   public RecyclingArrayListPubSub<PlanarRegionMessage> getPlanarRegions()
   {
      return planarRegions;
   }

   @Override
   public boolean epsilonEquals(PlanarRegionsListMessage other, double epsilon)
   {
      return MessageTools.epsilonEquals(planarRegions, other.planarRegions, epsilon);
   }
}

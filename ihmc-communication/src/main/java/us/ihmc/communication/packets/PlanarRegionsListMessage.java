package us.ihmc.communication.packets;

import us.ihmc.idl.PreallocatedList;

// FIXME Refactor to hold onto a single vertex buffer.
public class PlanarRegionsListMessage extends Packet<PlanarRegionsListMessage>
{
   public PreallocatedList<PlanarRegionMessage> planarRegions = new PreallocatedList<>(PlanarRegionMessage.class, PlanarRegionMessage::new, 500);

   public PlanarRegionsListMessage()
   {
   }

   @Override
   public void set(PlanarRegionsListMessage other)
   {
      MessageTools.copyData(other.planarRegions, planarRegions);
      setPacketInformation(other);
   }

   public PreallocatedList<PlanarRegionMessage> getPlanarRegions()
   {
      return planarRegions;
   }

   @Override
   public boolean epsilonEquals(PlanarRegionsListMessage other, double epsilon)
   {
      return MessageTools.epsilonEquals(planarRegions, other.planarRegions, epsilon);
   }
}

package us.ihmc.avatar.networkProcessor.modules.mocap;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.PlanarRegionsListMessage;

public class MocapPlanarRegionsListManager implements PacketConsumer<PlanarRegionsListMessage>
{
   private PlanarRegionsListMessage latestPlanarRegionsListMessage;
   private PlanarRegionsListMessage beforeWalkingPlanarRegionsListMessage;
   private PlanarRegionsListMessage afterWalkingPlanarRegionsListMessage;
   
   @Override
   public void receivedPacket(PlanarRegionsListMessage packet)
   {         
      this.latestPlanarRegionsListMessage = packet;
   }
   
   public void savePlanarRegionsBeforeWalking()
   {
      beforeWalkingPlanarRegionsListMessage = latestPlanarRegionsListMessage;
   }

   public void savePlanarRegionsAfterWalking()
   {
      afterWalkingPlanarRegionsListMessage = latestPlanarRegionsListMessage;
   }
   
   public PlanarRegionsListMessage getPlanarRegionsBeforeWalking()
   {
      return beforeWalkingPlanarRegionsListMessage;
   }

   public PlanarRegionsListMessage getPlanarRegionsAfterWalking()
   {
      return afterWalkingPlanarRegionsListMessage;
   }
}

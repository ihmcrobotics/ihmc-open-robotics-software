package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.wholebody.SingleJointAnglePacket;

public class SingleJointPositionProvider
{
   private final PacketConsumer<SingleJointAnglePacket> packetConsumer;
   private final AtomicReference<SingleJointAnglePacket> lastPacket = new AtomicReference<SingleJointAnglePacket>();
   
   public SingleJointPositionProvider()
   {
      packetConsumer = new PacketConsumer<SingleJointAnglePacket>()
      {
         @Override
         public void receivedPacket(SingleJointAnglePacket packet)
         {     
            if (packet != null)
            {
               lastPacket.set(packet);
            }
         }
      };
   }
   
   public boolean checkForNewPacket()
   {
      return lastPacket.get() != null;
   }
   
   public SingleJointAnglePacket getNewPacket()
   {
      return lastPacket.getAndSet(null);
   }
   
   public PacketConsumer<SingleJointAnglePacket> getPacketConsumer()
   {
      return packetConsumer;
   }
}

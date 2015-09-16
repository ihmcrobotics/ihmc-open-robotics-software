package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.JointAnglesPacket;

public class DesiredJointsPositionProvider 
{
   private final PacketConsumer<JointAnglesPacket>    jointsTargetPositionPacketConsumer;
   private final AtomicReference<JointAnglesPacket>   lastReceivedpacket = new AtomicReference<JointAnglesPacket>();
      
   public PacketConsumer<JointAnglesPacket> getPacketConsumer()
   {
      return jointsTargetPositionPacketConsumer;
   }
   
   public DesiredJointsPositionProvider()
   {

      jointsTargetPositionPacketConsumer = new PacketConsumer<JointAnglesPacket>()    {
         @Override
         public void receivedPacket(JointAnglesPacket packet)
         {     
            if (packet != null)
            {
               lastReceivedpacket.set(packet);
            }
         }
      };
   }

   public boolean checkForNewPacket()
   {
      return lastReceivedpacket.get() != null;
   }
   
   public JointAnglesPacket getNewPacket()
   {
      return lastReceivedpacket.getAndSet(null);
   }

}

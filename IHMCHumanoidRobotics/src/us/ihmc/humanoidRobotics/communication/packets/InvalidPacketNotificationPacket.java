package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

public class InvalidPacketNotificationPacket extends Packet<InvalidPacketNotificationPacket>
{
   public String packetClass;
   public String errorMessage;
   
   public InvalidPacketNotificationPacket()
   {
      setDestination(PacketDestination.BROADCAST);
   }
   
   public InvalidPacketNotificationPacket(Class<? extends Packet<?>> packetClass, String errorMessage)
   {
      this.packetClass = packetClass.getSimpleName();
      this.errorMessage = errorMessage;
   }

   @Override
   public boolean epsilonEquals(InvalidPacketNotificationPacket other, double epsilon)
   {
      return true;
   }

}

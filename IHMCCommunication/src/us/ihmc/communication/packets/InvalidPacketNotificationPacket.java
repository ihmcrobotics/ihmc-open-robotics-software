package us.ihmc.communication.packets;

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

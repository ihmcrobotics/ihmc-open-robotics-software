package us.ihmc.communication.packets;

public class InvalidPacketNotificationPacket extends SettablePacket<InvalidPacketNotificationPacket>
{
   public String packetClass;
   public String errorMessage;
   
   public InvalidPacketNotificationPacket()
   {
      setDestination(PacketDestination.BROADCAST);
   }
   
   public InvalidPacketNotificationPacket(Class<? extends Packet<?>> packetClass, String errorMessage)
   {
      set(packetClass, errorMessage);
   }

   public void set(Class<? extends Packet<?>> packetClass, String errorMessage)
   {
      this.packetClass = packetClass.getSimpleName();
      this.errorMessage = errorMessage;
   }

   @Override
   public void set(InvalidPacketNotificationPacket other)
   {
      this.packetClass = other.packetClass;
      this.errorMessage = other.errorMessage;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(InvalidPacketNotificationPacket other, double epsilon)
   {
      return true;
   }

}

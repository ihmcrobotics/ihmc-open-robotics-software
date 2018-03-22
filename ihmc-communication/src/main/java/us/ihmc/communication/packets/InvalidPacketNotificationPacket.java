package us.ihmc.communication.packets;

public class InvalidPacketNotificationPacket extends Packet<InvalidPacketNotificationPacket>
{
   public StringBuilder packetClass = new StringBuilder();
   public StringBuilder errorMessage = new StringBuilder();

   public InvalidPacketNotificationPacket()
   {
      setDestination(PacketDestination.BROADCAST);
   }

   public void set(Class<? extends Packet<?>> packetClass, String errorMessage)
   {
      this.packetClass.setLength(0);
      this.packetClass.append(packetClass.getSimpleName());
      this.errorMessage.setLength(0);
      this.errorMessage.append(errorMessage);
   }

   public String getPacketClassAsString()
   {
      return packetClass.toString();
   }

   public String getErrorMessageAsString()
   {
      return errorMessage.toString();
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

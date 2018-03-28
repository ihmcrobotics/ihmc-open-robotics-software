package us.ihmc.communication.packets;

public class InvalidPacketNotificationPacket extends Packet<InvalidPacketNotificationPacket>
{
   public StringBuilder packetClassSimpleName = new StringBuilder();
   public StringBuilder errorMessage = new StringBuilder();

   public InvalidPacketNotificationPacket()
   {
      setDestination(PacketDestination.BROADCAST);
   }

   public void setErrorMessage(String errorMessage)
   {
      this.errorMessage.setLength(0);
      this.errorMessage.append(errorMessage);
   }

   public void setPacketClassSimpleName(String packetClassSimpleName)
   {
      this.packetClassSimpleName.setLength(0);
      this.packetClassSimpleName.append(packetClassSimpleName);
   }

   public String getPacketClassSimpleNameAsString()
   {
      return packetClassSimpleName.toString();
   }

   public String getErrorMessageAsString()
   {
      return errorMessage.toString();
   }

   @Override
   public void set(InvalidPacketNotificationPacket other)
   {
      this.packetClassSimpleName = other.packetClassSimpleName;
      this.errorMessage = other.errorMessage;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(InvalidPacketNotificationPacket other, double epsilon)
   {
      return true;
   }
}

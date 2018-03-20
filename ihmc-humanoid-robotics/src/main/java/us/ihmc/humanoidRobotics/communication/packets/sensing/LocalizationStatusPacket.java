package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;

public class LocalizationStatusPacket extends Packet<LocalizationStatusPacket>
{
   public double overlap;
   public StringBuilder status = new StringBuilder();

   public LocalizationStatusPacket()
   {
      // for serialization
   }

   @Override
   public void set(LocalizationStatusPacket other)
   {
      overlap = other.overlap;
      status.setLength(0);
      status.append(other.status);
      setPacketInformation(other);
   }

   public double getOverlap()
   {
      return overlap;
   }

   public String getStatusAsString()
   {
      return status.toString();
   }

   @Override
   public boolean epsilonEquals(LocalizationStatusPacket other, double epsilon)
   {
      return (Math.abs(other.getOverlap() - this.getOverlap()) < epsilon) && (other.status.equals(this.status));
   }
}

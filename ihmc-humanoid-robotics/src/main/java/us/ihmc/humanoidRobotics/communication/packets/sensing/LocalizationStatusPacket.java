package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;

public class LocalizationStatusPacket extends Packet<LocalizationStatusPacket>
{
   public double overlap;
   public String status;

   public LocalizationStatusPacket()
   {
      // for serialization
   }

   @Override
   public void set(LocalizationStatusPacket other)
   {
      overlap = other.overlap;
      status = other.status;
      setPacketInformation(other);
   }

   public double getOverlap()
   {
      return overlap;
   }

   public String getStatus()
   {
      return status;
   }

   @Override
   public boolean epsilonEquals(LocalizationStatusPacket other, double epsilon)
   {
      return (Math.abs(other.getOverlap() - this.getOverlap()) < epsilon) && (other.getStatus().equals(this.getStatus()));
   }
}

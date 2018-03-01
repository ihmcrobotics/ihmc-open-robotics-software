package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

public class DrillDetectionPacket extends Packet<DrillDetectionPacket>
{
   public boolean isDrillOn;

   public DrillDetectionPacket()
   {
      isDrillOn = false;
      setDestination(PacketDestination.UI);
   }

   @Override
   public void set(DrillDetectionPacket other)
   {
      isDrillOn = other.isDrillOn;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(DrillDetectionPacket other, double epsilon)
   {
      return this.isDrillOn == other.isDrillOn;
   }
}

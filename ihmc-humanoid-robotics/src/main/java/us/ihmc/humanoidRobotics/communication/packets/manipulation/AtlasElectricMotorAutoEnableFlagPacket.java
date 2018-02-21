package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class AtlasElectricMotorAutoEnableFlagPacket extends Packet<AtlasElectricMotorAutoEnableFlagPacket>
{
   public boolean shouldAutoEnable;

   public AtlasElectricMotorAutoEnableFlagPacket()
   {
      setDestination(PacketDestination.CONTROLLER.ordinal());
   }

   @Override
   public void set(AtlasElectricMotorAutoEnableFlagPacket other)
   {
      setPacketInformation(other);
      shouldAutoEnable = other.shouldAutoEnable;
   }

   public boolean getShouldAutoEnable()
   {
      return shouldAutoEnable;
   }

   @Override
   public boolean epsilonEquals(AtlasElectricMotorAutoEnableFlagPacket other, double epsilon)
   {
      return other.shouldAutoEnable == shouldAutoEnable;
   }
}

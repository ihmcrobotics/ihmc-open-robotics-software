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

   }

   public AtlasElectricMotorAutoEnableFlagPacket(boolean shouldAutoEnable)
   {
      setDestination(PacketDestination.CONTROLLER.ordinal());
      this.shouldAutoEnable = shouldAutoEnable;
   }

   public boolean shouldAutoEnable()
   {
      return shouldAutoEnable;
   }

   @Override
   public boolean epsilonEquals(AtlasElectricMotorAutoEnableFlagPacket other, double epsilon)
   {
      return other.shouldAutoEnable == shouldAutoEnable;
   }
}

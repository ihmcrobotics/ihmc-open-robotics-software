package us.ihmc.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.PacketDestination;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
@ClassDocumentation(documentation = "Automates the process of powering each of the electric motors in the Atlas forearm")
public class AtlasElectricMotorAutoEnableFlagPacket extends IHMCRosApiPacket<AtlasElectricMotorAutoEnableFlagPacket>
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

   public AtlasElectricMotorAutoEnableFlagPacket(Random random)
   {
      this(random.nextBoolean());
   }

   public boolean shouldAutoEnable()
   {
      return shouldAutoEnable;
   }

   @Override public boolean epsilonEquals(AtlasElectricMotorAutoEnableFlagPacket other, double epsilon)
   {
      return other.shouldAutoEnable == shouldAutoEnable;
   }
}

package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.PacketDestination;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
@ClassDocumentation("Send a request to change the desired PSI of the Atlas hydraulic pump.")
public class AtlasDesiredPumpPSIPacket extends IHMCRosApiPacket<AtlasDesiredPumpPSIPacket>
{
   public int desiredPumpPSI;

   public AtlasDesiredPumpPSIPacket()
   {

   }

   public AtlasDesiredPumpPSIPacket(Random random)
   {
      this(random.nextInt());
   }

   public AtlasDesiredPumpPSIPacket(int desiredPumpPSI)
   {
      setDestination(PacketDestination.CONTROLLER.ordinal());
      this.desiredPumpPSI = desiredPumpPSI;
   }

   @Override public boolean epsilonEquals(AtlasDesiredPumpPSIPacket other, double epsilon)
   {
      return desiredPumpPSI == other.desiredPumpPSI;
   }
}

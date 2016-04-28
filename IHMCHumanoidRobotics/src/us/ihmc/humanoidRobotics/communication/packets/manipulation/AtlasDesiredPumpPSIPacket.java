package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
@RosMessagePacket(documentation = "Send a request to change the desired PSI of the Atlas hydraulic pump.",
      rosPackage = "ihmc_atlas",
      topic = "/control/desired_pump_psi")
public class AtlasDesiredPumpPSIPacket extends Packet<AtlasDesiredPumpPSIPacket>
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

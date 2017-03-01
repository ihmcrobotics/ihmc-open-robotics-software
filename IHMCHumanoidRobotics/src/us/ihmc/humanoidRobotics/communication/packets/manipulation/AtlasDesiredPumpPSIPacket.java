package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
@RosMessagePacket(documentation = "Send a request to change the desired PSI of the Atlas hydraulic pump.",
      rosPackage = "ihmc_atlas_ros",
      topic = "/control/desired_pump_psi")
public class AtlasDesiredPumpPSIPacket extends Packet<AtlasDesiredPumpPSIPacket>
{
   @RosExportedField(documentation = "The desired hydraulic pump PSI.")
   public int desiredPumpPsi;

   public AtlasDesiredPumpPSIPacket()
   {

   }

   public AtlasDesiredPumpPSIPacket(Random random)
   {
      this(random.nextInt());
   }

   public AtlasDesiredPumpPSIPacket(int desiredPumpPsi)
   {
      setDestination(PacketDestination.CONTROLLER.ordinal());
      this.desiredPumpPsi = desiredPumpPsi;
   }

   @Override public boolean epsilonEquals(AtlasDesiredPumpPSIPacket other, double epsilon)
   {
      return desiredPumpPsi == other.desiredPumpPsi;
   }
}

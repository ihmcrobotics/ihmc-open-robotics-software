package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;

/**
 * Created by agrabertilton on 4/28/15.
 */
@RosMessagePacket(documentation = "This message is used to abort walking, forcing the robot to switch back to double support and clear the footstep list.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/abort_walking")

public class AbortWalkingMessage extends Packet<AbortWalkingMessage>
{
   public AbortWalkingMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public AbortWalkingMessage(Random random)
   {
      this();
   }

   @Override
   public boolean epsilonEquals(AbortWalkingMessage other, double epsilon)
   {
      return true;
   }
}

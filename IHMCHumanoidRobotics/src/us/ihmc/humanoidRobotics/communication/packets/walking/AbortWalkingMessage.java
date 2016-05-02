package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.communication.packets.Packet;

import java.util.Random;

/**
 * Created by agrabertilton on 4/28/15.
 */
@RosMessagePacket(documentation = "This message is used to abort walking, forcing the robot to switch back to double support and clear the footstep list.",
                  rosPackage = "ihmc_msgs",
                  topic = "/control/abort_walking")

public class AbortWalkingMessage extends Packet<AbortWalkingMessage>
{
   public AbortWalkingMessage(Random random)
   {
      super(); // Do nothing, needed to make some unit tests work though.
   }

   @Override
   public boolean epsilonEquals(AbortWalkingMessage other, double epsilon)
   {
      return true;
   }
}

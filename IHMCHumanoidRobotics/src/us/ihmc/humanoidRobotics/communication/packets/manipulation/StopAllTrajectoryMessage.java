package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;

@RosMessagePacket(documentation = "Stop the execution of any trajectory being executed."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/stop_all_trajectories")
public class StopAllTrajectoryMessage extends Packet<StopAllTrajectoryMessage>
{

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public StopAllTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * 
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param random
    */
   public StopAllTrajectoryMessage(Random random)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public boolean epsilonEquals(StopAllTrajectoryMessage other, double epsilon)
   {
      return true;
   }
}

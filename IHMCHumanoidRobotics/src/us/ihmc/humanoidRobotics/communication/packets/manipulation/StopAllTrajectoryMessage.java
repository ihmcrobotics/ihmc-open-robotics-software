package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.communication.packets.Packet;

@ClassDocumentation("Stop the execution of any trajectory being executed."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.")
public class StopAllTrajectoryMessage extends IHMCRosApiMessage<StopAllTrajectoryMessage>
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

package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;

@RosMessagePacket(documentation = "This message pauses the execution of a list of footsteps. If this message is\n"
      + "sent in the middle of executing a footstep, the robot will finish the step and\n" + "pause when back in double support."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/pause_walking")
public class PauseWalkingMessage extends Packet<PauseWalkingMessage>
{
   @RosExportedField(documentation = "True to pause walking, false to unpause and resume an existing plan.")
   public boolean pause;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public PauseWalkingMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * 
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param pause
    */
   public PauseWalkingMessage(boolean pause)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.pause = pause;
   }

   public boolean isPaused()
   {
      return pause;
   }

   @Override
   public String toString()
   {
      return ("Paused = " + this.isPaused());
   }

   public boolean equals(PauseWalkingMessage obj)
   {
      return (this.isPaused() == obj.isPaused());
   }

   @Override
   public boolean equals(Object obj)
   {
      return ((obj instanceof PauseWalkingMessage) && this.equals((PauseWalkingMessage) obj));
   }

   @Override
   public boolean epsilonEquals(PauseWalkingMessage other, double epsilon)
   {
      return (this.isPaused() == other.isPaused());
   }

   public PauseWalkingMessage(Random random)
   {
      this(random.nextBoolean());
   }
}

package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;

@ClassDocumentation("This message pauses the execution of a list of footsteps. If this message is\n"
      + "sent in the middle of executing a footstep, the robot will finish the step and\n" + "pause when back in double support.")
public class PauseWalkingMessage extends IHMCRosApiPacket<PauseWalkingMessage>
{
   public boolean pause;

   /**
    * Empty constructor for serialization.
    */
   public PauseWalkingMessage()
   {
   }

   public PauseWalkingMessage(boolean pause)
   {
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

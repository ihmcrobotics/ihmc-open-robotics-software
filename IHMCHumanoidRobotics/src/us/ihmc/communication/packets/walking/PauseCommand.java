package us.ihmc.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;

@ClassDocumentation("This message pauses the execution of a list of footsteps. If this message is\n"
                                  + "sent in the middle of executing a footstep, the robot will finish the step and\n"
                                  + "pause when back in double support.")
public class PauseCommand extends IHMCRosApiPacket<PauseCommand>
{
   public boolean pause;
   
   public PauseCommand(boolean pause)
   {
      this.pause = pause;
   }
   
   public PauseCommand()
   {
      
   }
   
   public boolean isPaused()
   {
      return pause;
   }


   public String toString()
   {
	   return ("Paused = " + this.isPaused());
   }
   
   public boolean equals(PauseCommand obj)
   {
	   return (this.isPaused() == obj.isPaused());
   }
   
   public boolean equals(Object obj)
   {
	   return ((obj instanceof PauseCommand) && this.equals((PauseCommand)obj));
   }

    @Override
    public boolean epsilonEquals(PauseCommand other, double epsilon) {
        return (this.isPaused() == other.isPaused());
    }

    public PauseCommand(Random random)
    {
        this(random.nextBoolean());
    }
}

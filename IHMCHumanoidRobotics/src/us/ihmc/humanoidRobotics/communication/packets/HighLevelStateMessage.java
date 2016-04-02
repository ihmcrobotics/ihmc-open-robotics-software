package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;

@ClassDocumentation("This message is used to switch the control scheme between force and position control.\n"
                                  + "WARNING: When in position control, the IHMC balance algorithms will be disabled and\n"
                                  + "it is up to the user to ensure stability.")
public class HighLevelStateMessage extends Packet<HighLevelStateMessage>
{
   public HighLevelState highLevelState;

   public HighLevelStateMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public HighLevelStateMessage(HighLevelState highLevelState)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.highLevelState = highLevelState;
   }

   public HighLevelState getHighLevelState()
   {
      return highLevelState;
   }

   public boolean equals(Object obj)
   {
      return ((obj instanceof HighLevelStateMessage) && this.epsilonEquals((HighLevelStateMessage) obj, 0));
   }

   public String toString()
   {
      return "State= " + highLevelState.toString();
   }

   @Override
   public boolean epsilonEquals(HighLevelStateMessage other, double epsilon)
   {
      return this.getHighLevelState().equals(other.getHighLevelState());
   }

   public HighLevelStateMessage(Random random)
   {
      double value = random.nextInt(3);
      HighLevelState highLevelState = HighLevelState.WALKING;
      if (value == 1)
         highLevelState = HighLevelState.DRIVING;
      else if (value == 2)
         highLevelState = HighLevelState.INGRESS_EGRESS;

      this.highLevelState = highLevelState;
   }
}

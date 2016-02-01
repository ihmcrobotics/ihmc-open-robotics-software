package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;

@ClassDocumentation("This message is used to switch the control scheme between force and position control.\n"
                                  + "WARNING: When in position control, the IHMC balance algorithms will be disabled and\n"
                                  + "it is up to the user to ensure stability.")
public class HighLevelStatePacket extends IHMCRosApiPacket<HighLevelStatePacket>
{
   public us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState highLevelState;

   public HighLevelStatePacket()
   {
      // Empty constructor for deserialization
   }

   public HighLevelStatePacket(HighLevelState highLevelState)
   {
      this.highLevelState = highLevelState;
   }

   public HighLevelState getHighLevelState()
   {
      return highLevelState;
   }

   public boolean equals(Object obj)
   {
      return ((obj instanceof HighLevelStatePacket) && this.epsilonEquals((HighLevelStatePacket) obj, 0));
   }

   public String toString()
   {
      return "State= " + highLevelState.toString();
   }

   @Override
   public boolean epsilonEquals(HighLevelStatePacket other, double epsilon)
   {
      return this.getHighLevelState().equals(other.getHighLevelState());
   }

   public HighLevelStatePacket(Random random)
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

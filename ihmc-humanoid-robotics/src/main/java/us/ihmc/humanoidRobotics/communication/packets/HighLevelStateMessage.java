package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

@RosMessagePacket(documentation = "This message is used to switch the control scheme between force and position control.\n"
      + "WARNING: When in position control, the IHMC balance algorithms will be disabled and\n" + "it is up to the user to ensure stability.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/high_level_state")
public class HighLevelStateMessage extends Packet<HighLevelStateMessage>
{
   @RosExportedField(documentation = "The enum value of the current high level state of the robot.")
   public HighLevelControllerName highLevelState;

   public HighLevelStateMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public HighLevelStateMessage(HighLevelControllerName highLevelControllerName)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.highLevelState = highLevelControllerName;
   }

   public HighLevelControllerName getHighLevelControllerName()
   {
      return highLevelState;
   }

   @Override
   public boolean equals(Object obj)
   {
      return ((obj instanceof HighLevelStateMessage) && this.epsilonEquals((HighLevelStateMessage) obj, 0));
   }

   @Override
   public String toString()
   {
      return "State= " + highLevelState.toString();
   }

   @Override
   public boolean epsilonEquals(HighLevelStateMessage other, double epsilon)
   {
      return this.getHighLevelControllerName().equals(other.getHighLevelControllerName());
   }

   public HighLevelStateMessage(Random random)
   {
      double value = random.nextInt(3);
      HighLevelControllerName highLevelControllerName = HighLevelControllerName.WALKING;
      if (value == 1)
         highLevelControllerName = HighLevelControllerName.DO_NOTHING_BEHAVIOR;
      else if (value == 2)
         highLevelControllerName = HighLevelControllerName.DIAGNOSTICS;

      this.highLevelState = highLevelControllerName;
   }
}

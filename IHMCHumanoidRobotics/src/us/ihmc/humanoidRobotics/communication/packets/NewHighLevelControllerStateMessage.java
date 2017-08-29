package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelStates;

import java.util.Random;

@RosMessagePacket(documentation = "This message is used to switch the control scheme between force and position control.\n"
      + "WARNING: When in position control, the IHMC balance algorithms will be disabled and\n" + "it is up to the user to ensure stability.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/high_level_state")
public class NewHighLevelControllerStateMessage extends Packet<NewHighLevelControllerStateMessage>
{
   @RosExportedField(documentation = "The enum value of the current high level state of the robot.")
   public NewHighLevelStates highLevelState;

   public NewHighLevelControllerStateMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public NewHighLevelControllerStateMessage(NewHighLevelStates highLevelState)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.highLevelState = highLevelState;
   }

   public NewHighLevelStates getHighLevelState()
   {
      return highLevelState;
   }

   public boolean equals(Object obj)
   {
      return ((obj instanceof NewHighLevelControllerStateMessage) && this.epsilonEquals((NewHighLevelControllerStateMessage) obj, 0));
   }

   public String toString()
   {
      return "State= " + highLevelState.toString();
   }

   @Override
   public boolean epsilonEquals(NewHighLevelControllerStateMessage other, double epsilon)
   {
      return this.getHighLevelState().equals(other.getHighLevelState());
   }

   public NewHighLevelControllerStateMessage(Random random)
   {
      double value = random.nextInt(3);
      NewHighLevelStates highLevelState = NewHighLevelStates.WALKING_STATE;
      if (value == 1)
         highLevelState = NewHighLevelStates.DO_NOTHING_STATE;
      else if (value == 2)
         highLevelState = NewHighLevelStates.DIAGNOSTICS;

      this.highLevelState = highLevelState;
   }
}

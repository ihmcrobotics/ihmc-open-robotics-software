package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;

public class ModifiableChestTrajectoryMessage extends ModifiableSO3TrajectoryMessage<ModifiableChestTrajectoryMessage, ChestTrajectoryMessage>
{
   public ModifiableChestTrajectoryMessage()
   {
      
   }

   @Override
   public Class<ChestTrajectoryMessage> getMessageClass()
   {
      return ChestTrajectoryMessage.class;
   }
}

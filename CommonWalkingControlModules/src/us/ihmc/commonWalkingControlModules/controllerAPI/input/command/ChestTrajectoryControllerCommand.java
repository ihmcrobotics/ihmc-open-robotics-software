package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;

public class ChestTrajectoryControllerCommand extends SO3TrajectoryControllerCommand<ChestTrajectoryControllerCommand, ChestTrajectoryMessage>
{
   public ChestTrajectoryControllerCommand()
   {
      
   }

   @Override
   public Class<ChestTrajectoryMessage> getMessageClass()
   {
      return ChestTrajectoryMessage.class;
   }
}

package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;

public class ChestTrajectoryCommand extends SO3TrajectoryControllerCommand<ChestTrajectoryCommand, ChestTrajectoryMessage>
{
   public ChestTrajectoryCommand()
   {

   }

   @Override
   public Class<ChestTrajectoryMessage> getMessageClass()
   {
      return ChestTrajectoryMessage.class;
   }
}

package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;

import java.util.Random;

public class ChestTrajectoryCommand extends SO3TrajectoryControllerCommand<ChestTrajectoryCommand, ChestTrajectoryMessage>
{
   public ChestTrajectoryCommand()
   {

   }

   public ChestTrajectoryCommand(Random random)
   {
      super(random);
   }

   @Override
   public Class<ChestTrajectoryMessage> getMessageClass()
   {
      return ChestTrajectoryMessage.class;
   }
}

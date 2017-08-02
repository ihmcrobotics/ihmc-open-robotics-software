package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;

import java.util.Random;

public class HeadTrajectoryCommand extends SO3TrajectoryControllerCommand<HeadTrajectoryCommand, HeadTrajectoryMessage>
{
   public HeadTrajectoryCommand()
   {
   }

   public HeadTrajectoryCommand(Random random)
   {
      super(random);
   }

   @Override
   public Class<HeadTrajectoryMessage> getMessageClass()
   {
      return HeadTrajectoryMessage.class;
   }
}

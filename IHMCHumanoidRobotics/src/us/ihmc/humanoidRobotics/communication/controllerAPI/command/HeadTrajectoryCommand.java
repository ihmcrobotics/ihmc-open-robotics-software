package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;

public class HeadTrajectoryCommand extends SO3TrajectoryControllerCommand<HeadTrajectoryCommand, HeadTrajectoryMessage>
{
   public HeadTrajectoryCommand()
   {
   }

   @Override
   public Class<HeadTrajectoryMessage> getMessageClass()
   {
      return HeadTrajectoryMessage.class;
   }
}

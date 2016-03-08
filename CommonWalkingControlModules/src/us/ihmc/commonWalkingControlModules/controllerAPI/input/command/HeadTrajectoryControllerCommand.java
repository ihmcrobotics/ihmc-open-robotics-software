package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;

public class HeadTrajectoryControllerCommand extends SO3TrajectoryControllerCommand<HeadTrajectoryControllerCommand, HeadTrajectoryMessage>
{
   public HeadTrajectoryControllerCommand()
   {
   }

   @Override
   public Class<HeadTrajectoryMessage> getMessageClass()
   {
      return HeadTrajectoryMessage.class;
   }
}

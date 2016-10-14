package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;

public class PelvisTrajectoryCommand extends SE3TrajectoryControllerCommand<PelvisTrajectoryCommand, PelvisTrajectoryMessage>
{
   public PelvisTrajectoryCommand()
   {
   }

   @Override
   public Class<PelvisTrajectoryMessage> getMessageClass()
   {
      return PelvisTrajectoryMessage.class;
   }
}

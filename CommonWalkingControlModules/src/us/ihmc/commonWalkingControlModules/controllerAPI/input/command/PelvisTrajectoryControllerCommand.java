package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;

public class PelvisTrajectoryControllerCommand extends SE3TrajectoryControllerCommand<PelvisTrajectoryControllerCommand, PelvisTrajectoryMessage>
{
   public PelvisTrajectoryControllerCommand()
   {
   }

   @Override
   public Class<PelvisTrajectoryMessage> getMessageClass()
   {
      return PelvisTrajectoryMessage.class;
   }
}

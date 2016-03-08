package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;

public class PelvisOrientationTrajectoryControllerCommand
      extends SO3TrajectoryControllerCommand<PelvisOrientationTrajectoryControllerCommand, PelvisOrientationTrajectoryMessage>
{
   public PelvisOrientationTrajectoryControllerCommand()
   {
   }

   @Override
   public Class<PelvisOrientationTrajectoryMessage> getMessageClass()
   {
      return PelvisOrientationTrajectoryMessage.class;
   }
}

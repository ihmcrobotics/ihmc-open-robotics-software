package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;

public class PelvisOrientationTrajectoryCommand
      extends SO3TrajectoryControllerCommand<PelvisOrientationTrajectoryCommand, PelvisOrientationTrajectoryMessage>
{
   public PelvisOrientationTrajectoryCommand()
   {
   }

   @Override
   public Class<PelvisOrientationTrajectoryMessage> getMessageClass()
   {
      return PelvisOrientationTrajectoryMessage.class;
   }
}

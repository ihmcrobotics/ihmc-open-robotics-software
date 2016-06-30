package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;

public class PelvisOrientationTrajectoryCommand extends SO3TrajectoryControllerCommand<PelvisOrientationTrajectoryCommand, PelvisOrientationTrajectoryMessage>
{
   public PelvisOrientationTrajectoryCommand()
   {
   }

   public void set(PelvisTrajectoryCommand pelvisTrajectoryCommand)
   {
      setIncludingFrame(pelvisTrajectoryCommand);
      setCommandId(pelvisTrajectoryCommand.getCommandId());
      setExecutionMode(pelvisTrajectoryCommand.getExecutionMode());
      setPreviousCommandId(pelvisTrajectoryCommand.getPreviousCommandId());
   }

   @Override
   public Class<PelvisOrientationTrajectoryMessage> getMessageClass()
   {
      return PelvisOrientationTrajectoryMessage.class;
   }
}

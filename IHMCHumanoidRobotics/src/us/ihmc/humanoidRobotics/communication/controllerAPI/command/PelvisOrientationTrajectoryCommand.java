package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

import java.util.Random;

public class PelvisOrientationTrajectoryCommand extends SO3TrajectoryControllerCommand<PelvisOrientationTrajectoryCommand, PelvisOrientationTrajectoryMessage>
{
   private boolean enableUserPelvisControlDuringWalking = false;

   public PelvisOrientationTrajectoryCommand()
   {
   }

   public PelvisOrientationTrajectoryCommand(Random random)
   {
      super(random);
   }

   @Override
   public void set(PelvisOrientationTrajectoryCommand other)
   {
      setEnableUserPelvisControlDuringWalking(other.isEnableUserPelvisControlDuringWalking());
      super.set(other);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, PelvisOrientationTrajectoryMessage message)
   {
      setEnableUserPelvisControlDuringWalking(message.isEnableUserPelvisControlDuringWalking());
      super.set(resolver, message);
   }

   @Override
   public void set(PelvisOrientationTrajectoryMessage message)
   {
      setEnableUserPelvisControlDuringWalking(message.isEnableUserPelvisControlDuringWalking());
      super.set(message);
   }

   /**
    * Allows setting this orientation {@link #SO3TrajectoryControllerCommand} trajectory command
    * from a pelvis pose {@link #SE3TrajectoryControllerCommand} trajectory command.
    */
   public void set(PelvisTrajectoryCommand command)
   {
      clear(command.getDataFrame());
      setTrajectoryFrame(command.getTrajectoryFrame());
      getTrajectoryPointList().setIncludingFrame(command.getTrajectoryPointList());
      setEnableUserPelvisControlDuringWalking(command.isEnableUserPelvisControlDuringWalking());
      setQueueableCommandVariables(command);
      setSelectionMatrix(command.getSelectionMatrix().getAngularPart());
   }

   public boolean isEnableUserPelvisControlDuringWalking()
   {
      return enableUserPelvisControlDuringWalking;
   }

   public void setEnableUserPelvisControlDuringWalking(boolean enableUserPelvisControlDuringWalking)
   {
      this.enableUserPelvisControlDuringWalking = enableUserPelvisControlDuringWalking;
   }

   @Override
   public Class<PelvisOrientationTrajectoryMessage> getMessageClass()
   {
      return PelvisOrientationTrajectoryMessage.class;
   }
}

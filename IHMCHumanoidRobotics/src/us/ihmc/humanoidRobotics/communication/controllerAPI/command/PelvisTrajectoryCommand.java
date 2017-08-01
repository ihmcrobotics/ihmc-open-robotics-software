package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

import java.util.Random;

public class PelvisTrajectoryCommand extends SE3TrajectoryControllerCommand<PelvisTrajectoryCommand, PelvisTrajectoryMessage>
{
   private boolean enableUserPelvisControlDuringWalking = false;

   public PelvisTrajectoryCommand()
   {
      super(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
   }

   public PelvisTrajectoryCommand(Random random)
   {
      super(ReferenceFrame.generateRandomReferenceFrame("dataFrame", random, ReferenceFrame.getWorldFrame()),
            ReferenceFrame.generateRandomReferenceFrame("trajectoryFrame", random, ReferenceFrame.getWorldFrame()));
   }

   @Override
   public void set(PelvisTrajectoryCommand other)
   {
      setEnableUserPelvisControlDuringWalking(other.isEnableUserPelvisControlDuringWalking());
      super.set(other);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, PelvisTrajectoryMessage message)
   {
      setEnableUserPelvisControlDuringWalking(message.isEnableUserPelvisControlDuringWalking());
      super.set(resolver, message);
   }

   @Override
   public void set(PelvisTrajectoryMessage message)
   {
      setEnableUserPelvisControlDuringWalking(message.isEnableUserPelvisControlDuringWalking());
      super.set(message);
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
   public Class<PelvisTrajectoryMessage> getMessageClass()
   {
      return PelvisTrajectoryMessage.class;
   }
}

package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class PelvisTrajectoryCommand extends SE3TrajectoryControllerCommand<PelvisTrajectoryCommand, PelvisTrajectoryMessage>
{
   private boolean enableUserPelvisControlDuringWalking = false;

   public PelvisTrajectoryCommand()
   {
      super(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
   }

   public PelvisTrajectoryCommand(Random random)
   {
      super(EuclidFrameRandomTools.nextReferenceFrame("dataFrame", random, ReferenceFrame.getWorldFrame()),
            EuclidFrameRandomTools.nextReferenceFrame("trajectoryFrame", random, ReferenceFrame.getWorldFrame()));
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

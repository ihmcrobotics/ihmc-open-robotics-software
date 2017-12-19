package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class PelvisTrajectoryCommand extends SE3TrajectoryControllerCommand<PelvisTrajectoryCommand, PelvisTrajectoryMessage>
{
   private boolean enableUserPelvisControl = false;
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
      setEnableUserPelvisControl(other.isEnableUserPelvisControl());
      super.set(other);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, PelvisTrajectoryMessage message)
   {
      setEnableUserPelvisControlDuringWalking(message.isEnableUserPelvisControlDuringWalking());
      setEnableUserPelvisControl(message.isEnableUserPelvisControl());
      super.set(resolver, message);
   }

   @Override
   public void set(PelvisTrajectoryMessage message)
   {
      setEnableUserPelvisControlDuringWalking(message.isEnableUserPelvisControlDuringWalking());
      setEnableUserPelvisControl(message.isEnableUserPelvisControl());
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

   public boolean isEnableUserPelvisControl()
   {
      return enableUserPelvisControl;
   }

   public void setEnableUserPelvisControl(boolean enableUserPelvisControl)
   {
      this.enableUserPelvisControl = enableUserPelvisControl;
   }

   @Override
   public Class<PelvisTrajectoryMessage> getMessageClass()
   {
      return PelvisTrajectoryMessage.class;
   }
}

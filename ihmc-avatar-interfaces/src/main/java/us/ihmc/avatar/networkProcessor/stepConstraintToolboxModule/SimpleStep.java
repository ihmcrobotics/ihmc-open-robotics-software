package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

class SimpleStep
{
   private final RobotSide swingSide;
   private final double swingDuration;
   private final double startTime;
   private final FramePose3D stepPose = new FramePose3D();

   public SimpleStep(FootstepStatusMessage statusMessage, double startTime)
   {
      this.swingSide = RobotSide.fromByte(statusMessage.getRobotSide());
      this.swingDuration = statusMessage.getSwingDuration();
      this.startTime = startTime;
      stepPose.getPosition().set(statusMessage.getDesiredFootPositionInWorld());
      stepPose.getOrientation().set(statusMessage.getDesiredFootOrientationInWorld());
   }

   public double getSwingDuration()
   {
      return swingDuration;
   }

   public double getStartTime()
   {
      return startTime;
   }

   public RobotSide getSwingSide()
   {
      return swingSide;
   }

   public FramePose3DReadOnly getStepPose()
   {
      return stepPose;
   }
}

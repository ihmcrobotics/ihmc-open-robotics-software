package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCoPControlModule;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;

public class DoNothingVelocityViaCoPControlModule implements DesiredCoPControlModule
{
   private final CommonHumanoidReferenceFrames referenceFrames;

   public DoNothingVelocityViaCoPControlModule(CommonHumanoidReferenceFrames referenceFrames)
   {
      this.referenceFrames = referenceFrames;
   }

   public FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, FrameVector2d desiredVelocity, SingleSupportCondition singleSupportCondition, double timeInState)
   {
      return new FramePoint2d(referenceFrames.getAnkleZUpFrame(supportLeg));
   }

   public FramePoint2d computeDesiredCoPDoubleSupport(RobotSide loadingLeg, FrameVector2d desiredVelocity)
   {
      return new FramePoint2d(referenceFrames.getMidFeetZUpFrame());
   }

}

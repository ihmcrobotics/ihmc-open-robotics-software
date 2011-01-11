package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VelocityViaCoPControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;

public class DoNothingVelocityViaCoPControlModule implements VelocityViaCoPControlModule
{
   private final CommonWalkingReferenceFrames referenceFrames;

   public DoNothingVelocityViaCoPControlModule(CommonWalkingReferenceFrames referenceFrames)
   {
      this.referenceFrames = referenceFrames;
   }

   public FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, FrameVector2d desiredVelocity)
   {
      return new FramePoint2d(referenceFrames.getAnkleZUpFrame(supportLeg));
   }

   public FramePoint2d computeDesiredCoPDoubleSupport(RobotSide loadingLeg, FrameVector2d desiredVelocity)
   {
      return new FramePoint2d(referenceFrames.getMidFeetZUpFrame());
   }

}

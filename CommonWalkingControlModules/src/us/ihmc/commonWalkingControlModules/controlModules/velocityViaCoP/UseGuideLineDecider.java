package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;
import us.ihmc.robotics.geometry.FrameVector2d;

public interface UseGuideLineDecider
{
   public boolean useGuideLine(SingleSupportCondition singleSupportCondition, double timeInState, FrameVector2d desiredVelocity);
}

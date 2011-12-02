package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;
import us.ihmc.utilities.math.geometry.FrameVector2d;

public class MidwayIntoSwingUseGuideLineDecider implements UseGuideLineDecider
{

   public boolean useGuideLine(SingleSupportCondition singleSupportCondition, double timeInState, FrameVector2d desiredVelocity)
   {
      if (desiredVelocity.lengthSquared() == 0.0) return false;
      if (singleSupportCondition == SingleSupportCondition.StopWalking) return false;
      if (singleSupportCondition == SingleSupportCondition.Loading) return false;
      if ((singleSupportCondition == SingleSupportCondition.EarlyStance) && (timeInState < 0.2))
      {
         return false;
      }

      return true;
   }

}

package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;

public class NeverUseGuideLineDecider implements UseGuideLineDecider
{
   public boolean useGuideLine(SingleSupportCondition singleSupportCondition, double timeInState)
   {
      return false;
   }
}

package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;

public class AlwaysUseGuideLineDecider implements UseGuideLineDecider
{
   public boolean useGuideLine(SingleSupportCondition singleSupportCondition, double timeInState)
   {
      return true;
   }
}

package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;

public class MidwayIntoSwingUseGuideLineDecider implements UseGuideLineDecider
{

   public boolean useGuideLine(SingleSupportCondition singleSupportCondition, double timeInState)
   {
      if (singleSupportCondition == SingleSupportCondition.Loading) return false;
      if ((singleSupportCondition == SingleSupportCondition.EarlyStance) && (timeInState < 0.4))
      {
         return false;
      }

      return true;
   }

}

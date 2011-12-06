package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;
import us.ihmc.utilities.math.geometry.FrameVector2d;

public class MidwayIntoSwingUseGuideLineDecider implements UseGuideLineDecider
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final DoubleYoVariable earlyStanceWaitTime = new DoubleYoVariable("earlyStanceWaitTime", registry);

   public MidwayIntoSwingUseGuideLineDecider(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      earlyStanceWaitTime.set(0.2);
   }
   
   public boolean useGuideLine(SingleSupportCondition singleSupportCondition, double timeInState, FrameVector2d desiredVelocity)
   {
      if (desiredVelocity.lengthSquared() == 0.0) return false;
      if (singleSupportCondition == SingleSupportCondition.StopWalking) return false;
      if (singleSupportCondition == SingleSupportCondition.Loading) return false;
      if ((singleSupportCondition == SingleSupportCondition.EarlyStance) && (timeInState < earlyStanceWaitTime.getDoubleValue()))
      {
         return false;
      }

      return true;
   }

}

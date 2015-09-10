package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector2d;


public class MidwayIntoSwingUseGuideLineDecider implements UseGuideLineDecider
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final DoubleYoVariable earlyStanceWaitFraction = new DoubleYoVariable("earlyStanceWaitTime", registry);
   private final CouplingRegistry couplingRegistry;

   public MidwayIntoSwingUseGuideLineDecider(CouplingRegistry couplingRegistry, YoVariableRegistry parentRegistry)
   {
      this.couplingRegistry = couplingRegistry;
      parentRegistry.addChild(registry);
      earlyStanceWaitFraction.set(0.3);
   }
   
   public boolean useGuideLine(SingleSupportCondition singleSupportCondition, double timeInState, FrameVector2d desiredVelocity)
   {
      if (singleSupportCondition == SingleSupportCondition.StopWalking) return false;
      if (singleSupportCondition == SingleSupportCondition.Loading) return false;
      if ((singleSupportCondition == SingleSupportCondition.EarlyStance) && (timeInState < couplingRegistry.getSingleSupportDuration() * earlyStanceWaitFraction.getDoubleValue()))
      {
         return false;
      }

      return true;
   }

}

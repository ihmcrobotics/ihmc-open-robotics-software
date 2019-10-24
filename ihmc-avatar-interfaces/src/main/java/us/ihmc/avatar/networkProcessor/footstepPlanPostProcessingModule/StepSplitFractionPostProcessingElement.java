package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule;

import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class StepSplitFractionPostProcessingElement implements FootstepPlanPostProcessingElement
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoBoolean isActive = new YoBoolean("isActive", registry);

   public StepSplitFractionPostProcessingElement(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   @Override
   public boolean isActive()
   {
      return isActive.getBooleanValue();
   }

   @Override
   public void setIsActive(boolean isActive)
   {
      this.isActive.set(isActive);
   }

   @Override
   public FootstepPlanningToolboxOutputStatus postProcessFootstepPlan(FootstepPlanningToolboxOutputStatus outputStatus)
   {
      return outputStatus;
   }

   @Override
   public PostProcessingEnum getElementName()
   {
      return PostProcessingEnum.STEP_SPLIT_FRACTIONS;
   }
}

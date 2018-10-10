package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.SLOW)
public class PlanThenSnapPlannerToolboxTest extends FootstepPlannerToolboxTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.PLAN_THEN_SNAP;
   }
}

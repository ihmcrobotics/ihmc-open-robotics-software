package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;

public class FootstepPlanActionState extends BehaviorActionState
{
   private final FootstepPlanActionDefinition definition = new FootstepPlanActionDefinition();

   @Override
   public FootstepPlanActionDefinition getDefinition()
   {
      return definition;
   }
}

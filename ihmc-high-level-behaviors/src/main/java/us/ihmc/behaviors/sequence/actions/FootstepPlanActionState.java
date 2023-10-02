package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.behaviors.sequence.BehaviorActionStateBasics;

public class FootstepPlanActionState implements BehaviorActionState
{
   private final FootstepPlanActionDefinition definition = new FootstepPlanActionDefinition();
   private final BehaviorActionStateBasics stateBasics = new BehaviorActionStateBasics();

   @Override
   public long getID()
   {
      return stateBasics.getID();
   }

   @Override
   public FootstepPlanActionDefinition getDefinition()
   {
      return definition;
   }
}

package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;

public class FootstepActionState extends BehaviorActionState
{
   private final FootstepActionDefinition definition = new FootstepActionDefinition();

   @Override
   public FootstepActionDefinition getDefinition()
   {
      return definition;
   }
}

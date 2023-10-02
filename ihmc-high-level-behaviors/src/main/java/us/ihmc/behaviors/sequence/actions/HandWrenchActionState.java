package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;

public class HandWrenchActionState extends BehaviorActionState
{
   private final HandWrenchActionDefinition definition = new HandWrenchActionDefinition();

   @Override
   public HandWrenchActionDefinition getDefinition()
   {
      return definition;
   }
}

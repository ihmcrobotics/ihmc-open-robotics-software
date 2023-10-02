package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;

public class WaitDurationActionState extends BehaviorActionState
{
   private final WaitDurationActionDefinition definition = new WaitDurationActionDefinition();

   @Override
   public WaitDurationActionDefinition getDefinition()
   {
      return definition;
   }
}

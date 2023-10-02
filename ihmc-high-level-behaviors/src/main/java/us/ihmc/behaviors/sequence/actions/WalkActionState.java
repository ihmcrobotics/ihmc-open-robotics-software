package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;

public class WalkActionState extends BehaviorActionState
{
   private final WalkActionDefinition definition = new WalkActionDefinition();

   @Override
   public WalkActionDefinition getDefinition()
   {
      return definition;
   }
}

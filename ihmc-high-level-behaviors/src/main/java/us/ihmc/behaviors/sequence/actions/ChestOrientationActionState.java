package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;

public class ChestOrientationActionState extends BehaviorActionState
{
   private final ChestOrientationActionDefinition definition = new ChestOrientationActionDefinition();

   @Override
   public ChestOrientationActionDefinition getDefinition()
   {
      return definition;
   }
}

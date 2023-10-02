package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;

public class SakeHandCommandActionState extends BehaviorActionState
{
   private final SakeHandCommandActionDefinition definition = new SakeHandCommandActionDefinition();

   @Override
   public SakeHandCommandActionDefinition getDefinition()
   {
      return definition;
   }
}

package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;

public class PelvisHeightPitchActionState extends BehaviorActionState
{
   private final PelvisHeightPitchActionDefinition definition = new PelvisHeightPitchActionDefinition();

   @Override
   public PelvisHeightPitchActionDefinition getDefinition()
   {
      return definition;
   }
}

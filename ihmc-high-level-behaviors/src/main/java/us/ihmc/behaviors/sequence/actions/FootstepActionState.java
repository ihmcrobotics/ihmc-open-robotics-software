package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.behaviors.sequence.BehaviorActionStateBasics;

public class FootstepActionState extends FootstepActionDefinition implements BehaviorActionState
{
   private final FootstepActionDefinition definition = new FootstepActionDefinition();
   private final BehaviorActionStateBasics stateBasics = new BehaviorActionStateBasics();

   @Override
   public long getID()
   {
      return stateBasics.getID();
   }

   @Override
   public FootstepActionDefinition getDefinition()
   {
      return definition;
   }
}

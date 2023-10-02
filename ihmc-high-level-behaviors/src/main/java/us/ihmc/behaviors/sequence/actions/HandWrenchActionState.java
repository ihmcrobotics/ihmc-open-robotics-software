package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.behaviors.sequence.BehaviorActionStateBasics;

public class HandWrenchActionState extends HandWrenchActionDefinition implements BehaviorActionState
{
   private final HandWrenchActionDefinition definition = new HandWrenchActionDefinition();
   private final BehaviorActionStateBasics stateBasics = new BehaviorActionStateBasics();

   @Override
   public long getID()
   {
      return stateBasics.getID();
   }

   @Override
   public HandWrenchActionDefinition getDefinition()
   {
      return definition;
   }
}

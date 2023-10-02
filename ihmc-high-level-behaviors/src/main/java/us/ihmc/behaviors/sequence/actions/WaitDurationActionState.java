package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.behaviors.sequence.BehaviorActionStateBasics;

public class WaitDurationActionState implements BehaviorActionState
{
   private final WaitDurationActionDefinition definition = new WaitDurationActionDefinition();
   private final BehaviorActionStateBasics stateBasics = new BehaviorActionStateBasics();

   @Override
   public long getID()
   {
      return stateBasics.getID();
   }

   @Override
   public WaitDurationActionDefinition getDefinition()
   {
      return definition;
   }
}

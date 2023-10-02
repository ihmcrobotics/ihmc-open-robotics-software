package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.behaviors.sequence.BehaviorActionStateBasics;

public class WalkActionState extends WalkActionDefinition implements BehaviorActionState
{
   private final BehaviorActionStateBasics stateBasics = new BehaviorActionStateBasics();

   @Override
   public long getID()
   {
      return stateBasics.getID();
   }
}

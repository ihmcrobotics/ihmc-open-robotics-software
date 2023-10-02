package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.behaviors.sequence.BehaviorActionStateBasics;

public class ChestOrientationActionState implements BehaviorActionState
{
   private final ChestOrientationActionDefinition definition = new ChestOrientationActionDefinition();
   private final BehaviorActionStateBasics stateBasics = new BehaviorActionStateBasics();

   @Override
   public long getID()
   {
      return stateBasics.getID();
   }

   @Override
   public ChestOrientationActionDefinition getDefinition()
   {
      return definition;
   }
}

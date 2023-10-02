package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.behaviors.sequence.BehaviorActionStateBasics;

public class HandPoseActionState implements BehaviorActionState
{
   private final HandPoseActionDefinition definition = new HandPoseActionDefinition();
   private final BehaviorActionStateBasics stateBasics = new BehaviorActionStateBasics();

   @Override
   public long getID()
   {
      return stateBasics.getID();
   }

   @Override
   public HandPoseActionDefinition getDefinition()
   {
      return definition;
   }
}

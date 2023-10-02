package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.behaviors.sequence.BehaviorActionStateBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class HandPoseActionState implements BehaviorActionState
{
   private final HandPoseActionDefinition definition = new HandPoseActionDefinition();
   private final BehaviorActionStateBasics stateBasics = new BehaviorActionStateBasics();

   private ReferenceFrame palmFrame;

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

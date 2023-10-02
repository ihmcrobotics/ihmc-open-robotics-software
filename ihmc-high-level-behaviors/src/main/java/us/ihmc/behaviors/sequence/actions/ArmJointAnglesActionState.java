package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;

public class ArmJointAnglesActionState extends BehaviorActionState
{
   private final ArmJointAnglesActionDefinition definition = new ArmJointAnglesActionDefinition();

   @Override
   public ArmJointAnglesActionDefinition getDefinition()
   {
      return definition;
   }
}

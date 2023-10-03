package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;

public class ArmJointAnglesActionState extends BehaviorActionState<ArmJointAnglesActionDefinitionMessage>
{
   private final ArmJointAnglesActionDefinition definition = new ArmJointAnglesActionDefinition();

   @Override
   public ArmJointAnglesActionDefinition getDefinition()
   {
      return definition;
   }
}

package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;

public class ArmJointAnglesActionState extends BehaviorActionState
{
   private final ArmJointAnglesActionDefinition definition;

   public ArmJointAnglesActionState()
   {
      super(new ArmJointAnglesActionDefinition());
      this.definition = (ArmJointAnglesActionDefinition) super.getDefinition();
   }

   public void toMessage(ArmJointAnglesActionStateMessage message)
   {
      super.toMessage(message.getActionState());

      definition.toMessage(message.getDefinition());
   }

   public void fromMessage(ArmJointAnglesActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      definition.fromMessage(message.getDefinition());
   }

   @Override
   public ArmJointAnglesActionDefinition getDefinition()
   {
      return definition;
   }
}

package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;

public class ArmJointAnglesActionState extends BehaviorActionState
{
   private final ArmJointAnglesActionDefinition definition;

   public ArmJointAnglesActionState(long id, ArmJointAnglesActionDefinition definition)
   {
      super(id, definition);

      this.definition = definition;
   }

   public void toMessage(ArmJointAnglesActionStateMessage message)
   {
      super.toMessage(message.getActionState());
   }

   public void fromMessage(ArmJointAnglesActionStateMessage message)
   {
      super.fromMessage(message.getActionState());
   }

   @Override
   public ArmJointAnglesActionDefinition getDefinition()
   {
      return definition;
   }
}

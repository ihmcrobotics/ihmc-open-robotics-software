package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;

public class ArmJointAnglesActionState extends ActionNodeState<ArmJointAnglesActionDefinition>
{
   public ArmJointAnglesActionState(long id)
   {
      super(id, new ArmJointAnglesActionDefinition());
   }

   public void toMessage(ArmJointAnglesActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(ArmJointAnglesActionStateMessage message)
   {
      getDefinition().fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}

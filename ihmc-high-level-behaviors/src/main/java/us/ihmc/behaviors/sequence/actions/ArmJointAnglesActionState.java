package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;

public class ArmJointAnglesActionState extends BehaviorActionState<ArmJointAnglesActionDefinition>
{
   public ArmJointAnglesActionState(long id)
   {
      super(id, new ArmJointAnglesActionDefinition());
   }

   public void toMessage(ArmJointAnglesActionStateMessage message)
   {
      super.toMessage(message.getActionState());
   }

   public void fromMessage(ArmJointAnglesActionStateMessage message)
   {
      super.fromMessage(message.getActionState());
   }
}

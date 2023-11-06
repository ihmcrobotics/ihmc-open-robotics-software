package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;

public class ArmJointAnglesActionState extends ActionNodeState<ArmJointAnglesActionDefinition>
{
   public ArmJointAnglesActionState(long id, CRDTInfo crdtInfo)
   {
      super(id, new ArmJointAnglesActionDefinition(crdtInfo), crdtInfo);
   }

   public void toMessage(ArmJointAnglesActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(ArmJointAnglesActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }
}

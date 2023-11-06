package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandWrenchActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;

public class HandWrenchActionState extends ActionNodeState<HandWrenchActionDefinition>
{
   public HandWrenchActionState(long id, CRDTInfo crdtInfo)
   {
      super(id, new HandWrenchActionDefinition(crdtInfo), crdtInfo);
   }

   public void toMessage(HandWrenchActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(HandWrenchActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }
}

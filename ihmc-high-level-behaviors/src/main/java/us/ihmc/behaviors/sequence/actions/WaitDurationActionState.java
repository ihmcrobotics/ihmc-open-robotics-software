package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WaitDurationActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;

public class WaitDurationActionState extends ActionNodeState<WaitDurationActionDefinition>
{
   public WaitDurationActionState(long id, CRDTInfo crdtInfo)
   {
      super(id, new WaitDurationActionDefinition(crdtInfo), crdtInfo);
   }

   public void toMessage(WaitDurationActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(WaitDurationActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }
}

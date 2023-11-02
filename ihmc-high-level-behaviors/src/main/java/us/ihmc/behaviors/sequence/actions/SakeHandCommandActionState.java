package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SakeHandCommandActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;

public class SakeHandCommandActionState extends ActionNodeState<SakeHandCommandActionDefinition>
{
   public SakeHandCommandActionState(long id, CRDTInfo crdtInfo)
   {
      super(id, new SakeHandCommandActionDefinition(), crdtInfo);
   }

   public void toMessage(SakeHandCommandActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(SakeHandCommandActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }
}

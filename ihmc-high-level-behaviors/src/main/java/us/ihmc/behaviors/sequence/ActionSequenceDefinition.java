package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionSequenceDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;

public class ActionSequenceDefinition extends BehaviorTreeNodeDefinition
{
   // Seems to be nothing special here so far TODO Does that mean we delete it?

   public ActionSequenceDefinition(CRDTInfo crdtInfo)
   {
      super(crdtInfo);
   }

   public void toMessage(ActionSequenceDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(ActionSequenceDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}

package us.ihmc.behaviors.logic;

import behavior_msgs.msg.dds.ConditionNodeDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ConditionNodeDefinition extends BehaviorTreeNodeDefinition
{
   public ConditionNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);
   }

   public void toMessage(ConditionNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(ConditionNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}

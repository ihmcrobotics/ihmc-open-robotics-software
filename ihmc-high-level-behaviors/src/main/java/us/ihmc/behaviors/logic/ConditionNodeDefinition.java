package us.ihmc.behaviors.logic;

import behavior_msgs.msg.dds.ConditionNodeDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTGlobalInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ConditionNodeDefinition extends BehaviorTreeNodeDefinition
{
   public ConditionNodeDefinition(CRDTGlobalInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
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

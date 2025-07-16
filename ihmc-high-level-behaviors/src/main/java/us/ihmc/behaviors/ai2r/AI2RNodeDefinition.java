package us.ihmc.behaviors.ai2r;

import behavior_msgs.msg.dds.AI2RNodeDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class AI2RNodeDefinition extends BehaviorTreeNodeDefinition
{
   public AI2RNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);
   }

   public void toMessage(AI2RNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(AI2RNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}

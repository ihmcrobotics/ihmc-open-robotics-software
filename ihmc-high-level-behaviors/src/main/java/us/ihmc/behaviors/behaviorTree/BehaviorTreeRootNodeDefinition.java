package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeRootNodeDefinitionMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BehaviorTreeRootNodeDefinition extends BehaviorTreeNodeDefinition
{
   // Seems to be nothing special here so far TODO Does that mean we delete it?

   public BehaviorTreeRootNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);
   }

   public void toMessage(BehaviorTreeRootNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(BehaviorTreeRootNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}

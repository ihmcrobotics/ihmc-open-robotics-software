package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeRootNodeDefinitionMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BehaviorTreeRootNodeDefinition extends BehaviorTreeNodeDefinition
{
   private final WorkspaceResourceDirectory saveFileDirectory;

   public BehaviorTreeRootNodeDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo);

      this.saveFileDirectory = saveFileDirectory;
   }

   public void toMessage(BehaviorTreeRootNodeDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(BehaviorTreeRootNodeDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }

   public WorkspaceResourceDirectory getSaveFileDirectory()
   {
      return saveFileDirectory;
   }
}

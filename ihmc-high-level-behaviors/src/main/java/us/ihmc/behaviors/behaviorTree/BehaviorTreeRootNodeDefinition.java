package us.ihmc.behaviors.behaviorTree;

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

   public WorkspaceResourceDirectory getSaveFileDirectory()
   {
      return saveFileDirectory;
   }
}

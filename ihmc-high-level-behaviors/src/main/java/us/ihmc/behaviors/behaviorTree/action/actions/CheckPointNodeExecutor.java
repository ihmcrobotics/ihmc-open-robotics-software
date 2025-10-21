package us.ihmc.behaviors.behaviorTree.action.actions;

import us.ihmc.behaviors.behaviorTree.action.LeafNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class CheckPointNodeExecutor extends LeafNodeExecutor<CheckPointNodeState, CheckPointNodeDefinition>
{
   public CheckPointNodeExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new CheckPointNodeState(id, crdtInfo, saveFileDirectory));
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(false); // Completes immediately
   }
}

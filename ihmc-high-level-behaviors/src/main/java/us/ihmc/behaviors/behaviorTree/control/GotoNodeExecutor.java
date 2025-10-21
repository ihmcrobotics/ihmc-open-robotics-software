package us.ihmc.behaviors.behaviorTree.control;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.behaviorTree.LeafNodeExecutor;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class GotoNodeExecutor extends LeafNodeExecutor<GotoNodeState, GotoNodeDefinition>
{
   public GotoNodeExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new GotoNodeState(id, crdtInfo, saveFileDirectory));
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      if (!definition.getGotoNextNode())
      {
         LeafNodeState<?> nodeToGoto = state.findNodeToGoto();
         BehaviorTreeTools.findRootNode(this).getState().setExecutionNextIndex(nodeToGoto.getLeafIndex());
      }

      state.setIsExecuting(false); // Completes immediately
   }
}

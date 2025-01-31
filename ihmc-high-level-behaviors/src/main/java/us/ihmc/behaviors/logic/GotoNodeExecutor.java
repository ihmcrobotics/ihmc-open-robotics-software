package us.ihmc.behaviors.logic;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.LeafNodeExecutor;
import us.ihmc.behaviors.sequence.LeafNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class GotoNodeExecutor extends LeafNodeExecutor<GotoNodeState, GotoNodeDefinition>
{
   private final GotoNodeState state;
   private final GotoNodeDefinition definition;

   public GotoNodeExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new GotoNodeState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(false); // Completes immediately
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      if (!definition.getGotoNext().getValue())
      {
         LeafNodeState<?> nodeToGoto = state.findNodeToGoto();
         BehaviorTreeTools.findRootNode(this).getState().setExecutionNextIndex(nodeToGoto.getLeafIndex());
      }
   }
}

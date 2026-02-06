package us.ihmc.behaviors.behaviorTree.control;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.LeafNodeExecutor;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;

public class GotoNodeExecutor extends LeafNodeExecutor<GotoNodeState, GotoNodeDefinition>
{
   public GotoNodeExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new GotoNodeState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      if (!definition.getGotoNextNode())
      {
         LeafNodeState<?> nodeToGoto = state.findNodeToGoto();
         rootNode.getState().setExecutionNextIndex(nodeToGoto.getLeafIndex());
      }

      state.setIsExecuting(false); // Completes immediately
   }
}

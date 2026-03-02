package us.ihmc.behaviors.behaviorTree.control;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.LeafNodeExecutor;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
import us.ihmc.log.LogTools;

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
         BehaviorTreeNodeState<?> nodeToGoto = state.findNodeToGoto();

         while (!(nodeToGoto instanceof LeafNodeState) && nodeToGoto.getDepthFirstIndex() < rootNode.getState().getOrderedNodes().size() - 1)
            nodeToGoto = rootNode.getState().getOrderedNodes().get(nodeToGoto.getDepthFirstIndex() + 1);

         if (nodeToGoto instanceof LeafNodeState leafNodeState)
            rootNode.getState().setExecutionNextIndex(leafNodeState.getLeafIndex());
         else
         {
            LogTools.error("Cannot goto node %s. Reach end of tree.", nodeToGoto.getDefinition().getName());
            state.setFailed(true);
         }
      }

      state.setIsExecuting(false); // Completes immediately
   }
}

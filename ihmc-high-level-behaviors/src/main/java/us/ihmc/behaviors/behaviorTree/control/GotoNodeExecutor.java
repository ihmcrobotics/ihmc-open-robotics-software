package us.ihmc.behaviors.behaviorTree.control;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.LeafNodeExecutor;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;

public class GotoNodeExecutor extends LeafNodeExecutor<GotoNodeState, GotoNodeDefinition>
{
   private boolean failed;
   private boolean gotoNext;

   public GotoNodeExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new GotoNodeState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void triggerExecution()
   {
      gotoNext = true;
      failed = false;
      if (!definition.getGotoNextNode())
      {
         BehaviorTreeNodeState<?> nodeToGoto = state.findNodeToGoto();

         if (nodeToGoto != null) // Go through the sequence until
         {
            while (!(nodeToGoto instanceof LeafNodeState) && nodeToGoto.getDepthFirstIndex() < rootNode.getState().getOrderedNodes().size() - 1)
               nodeToGoto = rootNode.getState().getOrderedNodes().get(nodeToGoto.getDepthFirstIndex() + 1);

            if (nodeToGoto instanceof LeafNodeState<?> leafNodeState)
            {
               state.getLogger().info("Going to " + leafNodeState.getDefinition().getName());
               rootNode.getState().setExecutionNextIndex(leafNodeState.getLeafIndex());
               gotoNext = false;
            }
            else
            {
               state.getLogger().error("Cannot goto node %s. Reach end of tree.", nodeToGoto.getDefinition().getName());
               failed = true;
            }
         }
         else
         {
            state.getLogger().error("Cannot goto node %s. It does not exist.", definition.getNodeToGotoName());
            failed = true;
         }
      }

      if (gotoNext)
         rootNode.getState().stepForwardNextExecutionIndex();

      state.setFailed(failed);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(false); // Completes immediately
   }
}

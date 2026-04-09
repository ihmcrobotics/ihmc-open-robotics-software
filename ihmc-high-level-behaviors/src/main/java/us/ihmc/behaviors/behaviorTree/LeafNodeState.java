package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.LeafNodeStateMessage;
import us.ihmc.communication.crdt.CRDTStatusBoolean;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

import java.util.List;

public class LeafNodeState<D extends LeafNodeDefinition> extends BehaviorTreeNodeState<D>
{
   private final CRDTStatusBoolean isNextForExecution;
   private final CRDTStatusBoolean canExecute;
   private final CRDTStatusBoolean isExecuting;
   private final CRDTStatusBoolean failed;

   /** The index is not CRDT synced because it's a simple local calculation. */
   private int leafIndex = -1;

   public LeafNodeState(long id, D definition, BehaviorTreeRootNodeState rootNode)
   {
      super(id, definition, rootNode);

      isNextForExecution = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      canExecute = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, true);
      isExecuting = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      failed = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
   }

   public void validateDefinition(List<BehaviorTreeNodeState<?>> nodes)
   {
      if (definition.getExecuteAfterIsInvalid())
      {
         // We need to find the node by name
         // This happens when we load from JSON
         for (int j = depthFirstIndex - 1; j >= 0; j--) // Search backwards from previous
         {
            if (nodes.get(j).getDefinition().getName().equals(definition.getExecuteAfterLeafName()))
            {
               definition.setExecuteAfterLeaf(nodes.get(j).getID(), definition.getExecuteAfterLeafName());
               break;
            }
         }
      }
      else if (definition.getExecuteAfterNodeID() >= 0)
      {
         // Dynamically update the node name -- it can change independently of the node's ID
         // This is necessary for saving the definition
         for (int j = depthFirstIndex - 1; j >= 0; j--) // Search backwards from previous
            if (nodes.get(j).getID() == definition.getExecuteAfterNodeID())
               definition.setExecuteAfterLeafName(nodes.get(j).getDefinition().getName());
      }
   }

   @Override
   public boolean hasStatus()
   {
      boolean hasStatus = false;
      hasStatus |= isNextForExecution.pollHasStatus();
      hasStatus |= canExecute.pollHasStatus();
      hasStatus |= isExecuting.pollHasStatus();
      hasStatus |= failed.pollHasStatus();
      return hasStatus;
   }

   public void toMessage(LeafNodeStateMessage message)
   {
      super.toMessage(message.getState());

      message.setIsNextForExecution(isNextForExecution.toMessage());
      message.setCanExecute(canExecute.toMessage());
      message.setIsExecuting(isExecuting.toMessage());
      message.setFailed(failed.toMessage());
   }

   public void fromMessage(LeafNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      isNextForExecution.fromMessage(message.getIsNextForExecution());
      canExecute.fromMessage(message.getCanExecute());
      isExecuting.fromMessage(message.getIsExecuting());
      failed.fromMessage(message.getFailed());
   }

   public void setLeafIndex(int leafIndex)
   {
      this.leafIndex = leafIndex;
   }

   /** The index of the leaf, depth first over the entire tree. The first leaf is 0. */
   public int getLeafIndex()
   {
      return leafIndex;
   }

   public void setIsNextForExecution(boolean isNextForExecution)
   {
      this.isNextForExecution.setValue(isNextForExecution);
   }

   public boolean getIsNextForExecution()
   {
      return isNextForExecution.getValue();
   }

   /** Set from within {@link LeafNodeExecutor#update} only */
   public void setCanExecute(boolean canExecute)
   {
      this.canExecute.setValue(canExecute && !getExecuteAfterNodeIsMissing());
   }

   /** @return whether this leaf is valid for execution. This is checked before triggering the leaf.
    * Should be updated in the node's update() method. */
   public boolean getCanExecute()
   {
      return canExecute.getValue();
   }

   /** Set from within {@link LeafNodeExecutor#updateCurrentlyExecuting} only. */
   public void setIsExecuting(boolean isExecuting)
   {
      this.isExecuting.setValue(isExecuting);
   }

   public void setFailed(boolean failed)
   {
      this.failed.setValue(failed);
   }

   public boolean getFailed()
   {
      return failed.getValue();
   }

   /** Should return a precalculated value from {@link LeafNodeExecutor#updateCurrentlyExecuting} */
   public boolean getIsExecuting()
   {
      return isExecuting.getValue();
   }

   /** @return the index of the leaf to execute after as part of the concurrency system */
   public int getExecuteAfterLeafIndex()
   {
      if (definition.getExecuteAfterBeginning())
      {
         return -1;
      }
      else if (getExecuteAfterNodeIsMissing())
      {
         return leafIndex - 1; // Previous -- but this shouldn't actually happen, the action will fail first
      }
      else if (!definition.getExecuteAfterPrevious())
      {
         BehaviorTreeNodeState<?> executeAfterNode = rootNode.getIDToNodeMap().get(definition.getExecuteAfterNodeID());
         if (executeAfterNode != null)
         {
            if (executeAfterNode instanceof LeafNodeState<?> executeAfterLeaf)
               return executeAfterLeaf.getLeafIndex();
            else // Allow execute after to point to non-leaf nodes, in which case return previous to the next leaf
            {
               while (!(executeAfterNode instanceof LeafNodeState<?>) && executeAfterNode.getDepthFirstIndex() < rootNode.getOrderedNodes().size() - 1)
                  executeAfterNode = rootNode.getOrderedNodes().get(executeAfterNode.getDepthFirstIndex() + 1);
               if (executeAfterNode instanceof LeafNodeState<?> leafNodeState)
                  return leafNodeState.getLeafIndex() - 1;
            }
         }
      }

      return leafIndex - 1; // previous
   }

   public boolean getExecuteAfterNodeIsMissing()
   {
      return !definition.getExecuteAfterPrevious()
             && !definition.getExecuteAfterBeginning()
             && rootNode.getIDToNodeMap().get(definition.getExecuteAfterNodeID()) == null;
   }
}

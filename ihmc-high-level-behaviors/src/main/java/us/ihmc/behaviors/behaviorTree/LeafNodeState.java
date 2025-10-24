package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.LeafNodeStateMessage;
import us.ihmc.communication.crdt.CRDTStatusBoolean;
import us.ihmc.communication.crdt.CRDTStatusInteger;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.log.LogTools;

import java.util.List;

public class LeafNodeState<D extends LeafNodeDefinition> extends BehaviorTreeNodeState<D>
{
   private final CRDTStatusBoolean isNextForExecution;
   private final CRDTStatusInteger concurrencyRank;
   private final CRDTStatusBoolean canExecute;
   private final CRDTStatusBoolean isExecuting;
   private final CRDTStatusBoolean failed;

   /** The index is not CRDT synced because it's a simple local calculation. */
   private int leafIndex = -1;

   public LeafNodeState(long id, D definition, BehaviorTreeRootNodeState rootNode)
   {
      super(id, definition, rootNode);

      isNextForExecution = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      concurrencyRank = new CRDTStatusInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 1);
      canExecute = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, true);
      isExecuting = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      failed = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
   }

   public void validateFields(List<LeafNodeState<?>> leaves)
   {
      if (definition.getExecuteAfterIsInvalid())
      {
         // We need to find the node by name
         // This happens when we load from JSON
         for (int j = leafIndex - 1; j >= 0; j--) // Search backwards from previous
         {
            if (leaves.get(j).getDefinition().getName().equals(definition.getExecuteAfterLeafName()))
            {
               definition.setExecuteAfterLeaf(leaves.get(j).getID(), definition.getExecuteAfterLeafName());
               break;
            }
         }
      }
      else if (definition.getExecuteAfterNodeID() >= 0)
      {
         // Dynamically update the node name -- it can change independently of the node's ID
         // This is necessary for saving the definition
         for (int j = leafIndex - 1; j >= 0; j--) // Search backwards from previous
         {
            if (leaves.get(j).getID() == definition.getExecuteAfterNodeID())
            {
               definition.setExecuteAfterLeafName(leaves.get(j).getDefinition().getName());
            }
         }
      }
   }

   @Override
   public boolean hasStatus()
   {
      boolean hasStatus = false;
      hasStatus |= isNextForExecution.pollHasStatus();
      hasStatus |= concurrencyRank.pollHasStatus();
      hasStatus |= canExecute.pollHasStatus();
      hasStatus |= isExecuting.pollHasStatus();
      hasStatus |= failed.pollHasStatus();
      return hasStatus;
   }

   public void toMessage(LeafNodeStateMessage message)
   {
      super.toMessage(message.getState());

      message.setIsNextForExecution(isNextForExecution.toMessage());
      message.setConcurrencyRank(concurrencyRank.toMessage());
      message.setCanExecute(canExecute.toMessage());
      message.setIsExecuting(isExecuting.toMessage());
      message.setFailed(failed.toMessage());
   }

   public void fromMessage(LeafNodeStateMessage message)
   {
      super.fromMessage(message.getState());

      isNextForExecution.fromMessage(message.getIsNextForExecution());
      concurrencyRank.fromMessage(message.getConcurrencyRank());
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

   public void setConcurrencyRank(int concurrencyRank)
   {
      this.concurrencyRank.setValue(concurrencyRank);
   }

   /**
    * Gives an idea how many leaves will be executing all together with this one.
    * How many leaves will be started when the execute next index is set to this action.
    */
   public int getConcurrencyRank()
   {
      return concurrencyRank.getValue();
   }

   public boolean getIsToBeExecutedConcurrently()
   {
      return concurrencyRank.getValue() > 1;
   }

   public void setCanExecute(boolean canExecute)
   {
      this.canExecute.setValue(canExecute);
   }

   /** @return whether this leaf is valid for execution. This is checked before triggering the leaf. */
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
      else if (!definition.getExecuteAfterPrevious())
      {
         LeafNodeState<?> executeAfterNode = getExecuteAfterLeaf();

         if (executeAfterNode != null)
            return executeAfterNode.getLeafIndex();
      }

      return leafIndex - 1; // previous
   }

   /** @return the leaf to execute after as part of the concurrency system */
   public LeafNodeState<?> getExecuteAfterLeaf()
   {
      if (BehaviorTreeTools.findRootNode(this).getIDToNodeMap().get(definition.getExecuteAfterNodeID()) instanceof LeafNodeState<?> executeAfterNode)
      {
         return executeAfterNode;
      }

      LogTools.error("Node ID not found: {}", definition.getExecuteAfterNodeID());

      return null;
   }
}

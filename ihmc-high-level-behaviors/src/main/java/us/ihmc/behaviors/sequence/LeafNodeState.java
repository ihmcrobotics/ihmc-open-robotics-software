package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.LeafNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusBoolean;
import us.ihmc.communication.crdt.CRDTStatusInteger;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.log.LogTools;

import java.util.List;

public class LeafNodeState<D extends LeafNodeDefinition> extends BehaviorTreeNodeState<D>
{
   private final D definition;

   private final CRDTStatusBoolean isNextForExecution;
   private final CRDTStatusInteger concurrencyRank;
   private final CRDTStatusBoolean canExecute;
   private final CRDTStatusBoolean isExecuting;
   private final CRDTStatusBoolean failed;

   /** The index is not CRDT synced because it's a simple local calculation. */
   private int actionIndex = -1;

   public LeafNodeState(long id, D definition, CRDTInfo crdtInfo)
   {
      super(id, definition, crdtInfo);

      this.definition = definition;

      isNextForExecution = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      concurrencyRank = new CRDTStatusInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 1);
      canExecute = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, true);
      isExecuting = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      failed = new CRDTStatusBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
   }

   /**
    * Updates the definition executeAfterNodeName string for
    * saving an up to date human readable name in the JSON.
    * It also finds the correct node upon loading the name from JSON.
    */
   public void updateAndValidateExecuteAfter(List<LeafNodeState<?>> actionStateChildren)
   {
      String executeAfterActionName = null;

      if (!definition.getExecuteAfterPrevious().getValue() && !definition.getExecuteAfterBeginning().getValue())
      {
         // We need to find the node by name
         // This happens when we load from JSON
         if (definition.getExecuteAfterNodeID().getValue() == 0)
         {
            for (int j = actionIndex - 1; j >= 0; j--)
            {
               LeafNodeState<?> stateToCompare = actionStateChildren.get(j);
               if (stateToCompare.getDefinition().getName().equals(definition.getExecuteAfterActionName()))
               {
                  executeAfterActionName = stateToCompare.getDefinition().getName();
                  definition.getExecuteAfterNodeID().setValue(stateToCompare.getID());
                  break;
               }
            }
         }
         else // Update the node's name for saving in human readable format
         {
            long executeAfterID = definition.getExecuteAfterNodeID().getValue();
            for (int j = actionIndex - 1; j >= 0; j--)
            {
               LeafNodeState<?> actionStateToCompare = actionStateChildren.get(j);
               if (actionStateToCompare.getID() == executeAfterID)
               {
                  executeAfterActionName = actionStateToCompare.getDefinition().getName();
               }
            }
         }
      }

      definition.updateAndSanitizeExecuteAfterFields(executeAfterActionName);
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

   public void setActionIndex(int actionIndex)
   {
      this.actionIndex = actionIndex;
   }

   public int getActionIndex()
   {
      return actionIndex;
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
    * Gives an idea how many actions will be executing all together with this one.
    * How many actions will be started when the execute next index is set to this action.
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

   /** @return whether this action is valid for execution. This is checked before triggering the action. */
   public boolean getCanExecute()
   {
      return canExecute.getValue();
   }

   /** Set from within {@link ActionNodeExecutor#updateCurrentlyExecuting} only. */
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

   /** Should return a precalculated value from {@link ActionNodeExecutor#updateCurrentlyExecuting} */
   public boolean getIsExecuting()
   {
      return isExecuting.getValue();
   }

   public int calculateExecuteAfterActionIndex()
   {
      if (definition.getExecuteAfterBeginning().getValue())
      {
         return -1;
      }
      else if (!definition.getExecuteAfterPrevious().getValue())
      {
         LeafNodeState<?> executeAfterNode = findExecuteAfterAction();

         if (executeAfterNode != null)
            return executeAfterNode.getActionIndex();
      }

      return actionIndex - 1; // previous
   }

   public LeafNodeState<?> findExecuteAfterAction()
   {
      long executeAfterID = definition.getExecuteAfterNodeID().getValue();

      if (BehaviorTreeTools.findRootNode(this).getIDToNodeMap().get(executeAfterID) instanceof LeafNodeState<?> executeAfterNode)
      {
         return executeAfterNode;
      }

      LogTools.error("Node ID not found: {}", executeAfterID);

      return null;
   }
}

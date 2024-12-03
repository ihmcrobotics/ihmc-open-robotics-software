package us.ihmc.behaviors.behaviorTree;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

public class BehaviorTreeRootNodeExecutor extends BehaviorTreeNodeExecutor<BehaviorTreeRootNodeState, BehaviorTreeRootNodeDefinition>
{
   private final BehaviorTreeRootNodeState state;
   private final BehaviorTreeRootNodeDefinition definition;
   private final TLongObjectHashMap<BehaviorTreeNodeExecutor<?, ?>> idToNodeMap = new TLongObjectHashMap<>();
   private final List<ActionNodeExecutor<?, ?>> actionChildren = new ArrayList<>();
   private final List<ActionNodeExecutor<?, ?>> currentlyExecutingActions = new ArrayList<>();

   public BehaviorTreeRootNodeExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new BehaviorTreeRootNodeState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();
   }

   @Override
   public void tick()
   {
      super.tick();

      // TODO: Tick children
   }

   @Override
   public void update()
   {
      super.update();

      idToNodeMap.clear();
      actionChildren.clear();
      currentlyExecutingActions.clear();
      updateActionSubtree(this);

      for (ActionNodeExecutor<?, ?> actionChild : actionChildren)
      {
         actionChild.getState().updateAndValidateExecuteAfter(state.getActionChildren());
      }

      // Update concurrency ranks
      for (int i = 0; i < state.getActionChildren().size(); i++)
      {
         state.getActionChildren().get(i).setConcurrencyRank(1);

//         int j = i + 1;
//         for (; j < state.getActionChildren().size()
//              && state.getActionChildren().get(j).calculateExecuteAfterActionIndex(state.getActionChildren()) < i; j++);

         int j = i - 1;
         for (; j >= 0; j--)
         {
            int thisExecuteAfterActionIndex = state.getActionChildren().get(i).calculateExecuteAfterActionIndex(getState().getActionChildren());
            int executeAfterActionIndexToCompare = state.getActionChildren().get(j).calculateExecuteAfterActionIndex(getState().getActionChildren());
            if (thisExecuteAfterActionIndex == executeAfterActionIndexToCompare)
            {
               state.getActionChildren().get(i).setConcurrencyRank(2);
            }
         }
      }

      // Update is next for execution
      for (int i = 0; i < state.getActionChildren().size(); i++)
      {
         int executionNextIndex = state.getExecutionNextIndex();
         if (i < executionNextIndex)
         {
            state.getActionChildren().get(i).setIsNextForExecution(false);
         }
         else if (i == executionNextIndex)
         {
            state.getActionChildren().get(i).setIsNextForExecution(true);
         }
         else if (state.getActionChildren().get(i).calculateExecuteAfterActionIndex(state.getActionChildren()) < executionNextIndex)
         {
            state.getActionChildren().get(i).setIsNextForExecution(true);
         }
         else
         {
            state.getActionChildren().get(i).setIsNextForExecution(false);
         }
      }

      boolean anyActionExecutionFailed = false;
      for (ActionNodeExecutor<?, ?> currentlyExecutingAction : currentlyExecutingActions)
      {
         currentlyExecutingAction.updateCurrentlyExecuting();
         anyActionExecutionFailed |= currentlyExecutingAction.getState().getFailed();
      }

      if (state.getAutomaticExecution())
      {
         if (isEndOfSequence())
         {
            state.getLogger().info("End of sequence.");
            state.setAutomaticExecution(false);
         }
         else if (anyActionExecutionFailed)
         {
            state.getLogger().error("An action failed. Disabling automatic execution.");
            state.setAutomaticExecution(false);
         }
         else
         {
            while (shouldExecuteNextAction())
            {
               state.getLogger().info("Automatically executing action: {}", actionChildren.get(state.getExecutionNextIndex()).getClass().getSimpleName());
               executeNextAction();
            }
         }
      }
      else if (state.pollManualExecutionRequested())
      {
         while (shouldExecuteNextAction())
         {
            state.getLogger().info("Manually executing action: {}", actionChildren.get(state.getExecutionNextIndex()).getClass().getSimpleName());
            executeNextAction();
         }
      }
   }

   public void updateActionSubtree(BehaviorTreeNodeExecutor<?, ?> node)
   {
      idToNodeMap.put(node.getState().getID(), node);

      for (BehaviorTreeNodeExecutor<?, ?> child : node.getChildren())
      {
         if (child instanceof ActionNodeExecutor<?, ?> actionNode)
         {
            actionChildren.add(actionNode);
            if (actionNode.getState().getIsExecuting())
            {
               currentlyExecutingActions.add(actionNode);
            }
         }
         else
         {
            updateActionSubtree(child);
         }
      }
   }

   private void executeNextAction()
   {
      ActionNodeExecutor<?, ?> actionToExecute = actionChildren.get(state.getExecutionNextIndex());

      state.getLogger().info("Triggering action execution: %s".formatted(actionToExecute.getDefinition().getName()));
      actionToExecute.update();
      actionToExecute.triggerActionExecution();
      actionToExecute.updateCurrentlyExecuting();
      currentlyExecutingActions.add(actionToExecute);
      state.stepForwardNextExecutionIndex();
   }

   private boolean shouldExecuteNextAction()
   {
      if (isEndOfSequence())
      {
         return false;
      }

      ActionNodeExecutor<?, ?> nextNodeToExecute = actionChildren.get(state.getExecutionNextIndex());

      if (!nextNodeToExecute.getState().getCanExecute())
      {
         state.getLogger().error("Cannot execute action: %s\n%s".formatted(nextNodeToExecute.getDefinition().getName(),
                                                                           nextNodeToExecute.getCantExecuteMessage()));
         state.setAutomaticExecution(false);
         return false;
      }

      if (state.getConcurrencyEnabled())
      {
         int executeAfterActionIndex = nextNodeToExecute.getState().calculateExecuteAfterActionIndex(getState().getActionChildren());

         if (executeAfterActionIndex < 0) // Execute after beginning
         {
            return true;
         }
         else
         {
            return !actionChildren.get(executeAfterActionIndex).getState().getIsExecuting();
         }
      }
      else
      {
         boolean anyActionExecuting = false;
         for (ActionNodeExecutor<?, ?> executorChild : actionChildren)
         {
            anyActionExecuting |= executorChild.getState().getIsExecuting();
         }
         return  !anyActionExecuting;
      }
   }

   private boolean isEndOfSequence()
   {
      return state.getExecutionNextIndex() >= actionChildren.size();
   }
   
   public TLongObjectHashMap<BehaviorTreeNodeExecutor<?, ?>> getIDToNodeMap()
   {
      return idToNodeMap;
   }

   public List<ActionNodeExecutor<?, ?>> getActionChildren()
   {
      return actionChildren;
   }

   public List<ActionNodeExecutor<?, ?>> getCurrentlyExecutingActions()
   {
      return currentlyExecutingActions;
   }
}

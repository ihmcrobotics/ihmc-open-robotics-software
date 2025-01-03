package us.ihmc.behaviors.behaviorTree;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.FallbackNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

public class BehaviorTreeRootNodeExecutor extends BehaviorTreeNodeExecutor<BehaviorTreeRootNodeState, BehaviorTreeRootNodeDefinition>
{
   private final BehaviorTreeRootNodeState state;
   private final BehaviorTreeRootNodeDefinition definition;
   private final TLongObjectHashMap<BehaviorTreeNodeExecutor<?, ?>> idToNodeMap = new TLongObjectHashMap<>();
   private final List<ActionNodeExecutor<?, ?>> actionChildren = new ArrayList<>();
   private final List<FallbackNodeExecutor> fallbackNodes = new ArrayList<>();
   private final List<ActionNodeExecutor<?, ?>> currentlyExecutingActions = new ArrayList<>();
   private final List<ActionNodeExecutor<?, ?>> failedActions = new ArrayList<>();
   private final List<ActionNodeExecutor<?, ?>> successfulActions = new ArrayList<>();
   private final List<ActionNodeExecutor<?, ?>> failedActionsWithoutFallback = new ArrayList<>();

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
      fallbackNodes.clear();
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
            int thisExecuteAfterActionIndex = state.getActionChildren().get(i).calculateExecuteAfterActionIndex();
            int executeAfterActionIndexToCompare = state.getActionChildren().get(j).calculateExecuteAfterActionIndex();
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
         else if (state.getActionChildren().get(i).calculateExecuteAfterActionIndex() < executionNextIndex)
         {
            state.getActionChildren().get(i).setIsNextForExecution(true);
         }
         else
         {
            state.getActionChildren().get(i).setIsNextForExecution(false);
         }
      }

      failedActions.clear();
      successfulActions.clear();
      for (ActionNodeExecutor<?, ?> currentlyExecutingAction : currentlyExecutingActions)
      {
         currentlyExecutingAction.updateCurrentlyExecuting();

         // This action has completed on this update
         if (!currentlyExecutingAction.getState().getIsExecuting())
         {
            String name = currentlyExecutingAction.getDefinition().getName();
            double elapsedExecutionTime = currentlyExecutingAction.getState().getElapsedExecutionTime();
            if (currentlyExecutingAction.getState().getFailed())
            {
               failedActions.add(currentlyExecutingAction);
               LogTools.error("Action failed after %.2f s: %s".formatted(elapsedExecutionTime, name));
            }
            else
            {
               successfulActions.add(currentlyExecutingAction);
               LogTools.info("Action completed successfully in %.2f s: %s".formatted(elapsedExecutionTime, name));
            }
         }
      }
      currentlyExecutingActions.removeAll(failedActions);
      currentlyExecutingActions.removeAll(successfulActions);

      failedActionsWithoutFallback.clear();
      failedActionsWithoutFallback.addAll(failedActions);
      for (FallbackNodeExecutor fallbackNode : fallbackNodes)
      {
         fallbackNode.update();

         if (!fallbackNode.getFallbackActions().isEmpty())
         {
            failedActionsWithoutFallback.removeAll(fallbackNode.getTryActions());
         }
      }

      // Handle skipping the fallback if try action group is successful
      boolean actionEnded = !failedActions.isEmpty() || !successfulActions.isEmpty();
      if (actionEnded && currentlyExecutingActions.isEmpty() && !isEndOfSequence())
      {
         ActionNodeExecutor<?, ?> nextNodeToExecute = actionChildren.get(state.getExecutionNextIndex());

         for (FallbackNodeExecutor fallbackNode : fallbackNodes)
         {
            if (fallbackNode.getFallbackActions().indexOf(nextNodeToExecute) == 0)
            {
               boolean anyFailed = false;
               for (ActionNodeExecutor<?, ?> tryAction : fallbackNode.getTryActions())
               {
                  if (tryAction.getState().getFailed())
                  {
                     anyFailed = true;
                     break;
                  }
               }

               if (anyFailed) // Nothing is executing and a tried action failed -- falling back
               {
                  LogTools.warn("Actions failed, fallback actions are next for execution.");
               }
               else // Nothing is executing and none of the try actions failed -- skip fallback
               {
                  ActionNodeExecutor<?, ?> lastFallbackAction = fallbackNode.getFallbackActions().get(fallbackNode.getFallbackActions().size() - 1);
                  LogTools.info("Actions successful, skipping fallback actions(s)");
                  state.setExecutionNextIndex(lastFallbackAction.getState().getActionIndex() + 1);
               }
            }
         }
      }

      if (state.getAutomaticExecution())
      {
         if (isEndOfSequence())
         {
            state.getLogger().info("End of sequence.");
            state.setAutomaticExecution(false);
         }
         else if (!failedActionsWithoutFallback.isEmpty())
         {
            state.getLogger().error("An action failed. Disabling automatic execution.\n   Failed: %s".formatted(failedActionsWithoutFallback));
            state.setAutomaticExecution(false);
         }
         else
         {
            while (shouldExecuteNextAction())
            {
               var nextAction = actionChildren.get(state.getExecutionNextIndex());
               state.getLogger().info("Automatically executing action: %s (%s)".formatted(nextAction.getDefinition().getName(),
                                                                                          nextAction.getClass().getSimpleName()));
               executeNextAction();
            }
         }
      }
      else if (state.pollManualExecutionRequested())
      {
         while (shouldExecuteNextAction())
         {
            var nextAction = actionChildren.get(state.getExecutionNextIndex());
            state.getLogger().info("Manually executing action: %s (%s)".formatted(nextAction.getDefinition().getName(),
                                                                                  nextAction.getClass().getSimpleName()));
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
         else if (child instanceof FallbackNodeExecutor fallbackNode)
         {
            fallbackNodes.add(fallbackNode);
         }

         updateActionSubtree(child);
      }
   }

   private void executeNextAction()
   {
      ActionNodeExecutor<?, ?> actionToExecute = actionChildren.get(state.getExecutionNextIndex());

      state.getLogger().info("Triggering action execution: %s (%s)".formatted(actionToExecute.getDefinition().getName(),
                                                                              actionToExecute.getClass().getSimpleName()));
      actionToExecute.update();
      actionToExecute.triggerActionExecution();
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

      // If a fallback sequence is up next, block if any corresponding try actions are executing
      for (FallbackNodeExecutor fallbackNode : fallbackNodes)
      {
         if (fallbackNode.getFallbackActions().indexOf(nextNodeToExecute) == 0) // The first fallback action is next
         {
            for (ActionNodeExecutor<?, ?> tryAction : fallbackNode.getTryActions())
            {
               if (tryAction.getState().getIsExecuting())
               {
                  return false;
               }
            }
         }
      }

      if (!nextNodeToExecute.getState().getCanExecute())
      {
         state.getLogger().error("Cannot execute action: %s\n%s".formatted(nextNodeToExecute.getDefinition().getName(),
                                                                           nextNodeToExecute.getCantExecuteMessage()));
         state.setAutomaticExecution(false);
         return false;
      }

      if (state.getConcurrencyEnabled())
      {
         int executeAfterActionIndex = nextNodeToExecute.getState().calculateExecuteAfterActionIndex();

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
         return !currentlyExecutingActions.isEmpty();
      }
   }

   public boolean isEndOfSequence()
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

   public List<ActionNodeExecutor<?, ?>> getFailedActions()
   {
      return failedActions;
   }

   public List<ActionNodeExecutor<?, ?>> getSuccessfulActions()
   {
      return successfulActions;
   }

   public List<ActionNodeExecutor<?, ?>> getCurrentlyExecutingActions()
   {
      return currentlyExecutingActions;
   }
}

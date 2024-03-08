package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

public class ActionSequenceExecutor extends BehaviorTreeNodeExecutor<ActionSequenceState, ActionSequenceDefinition>
{
   private final ActionSequenceState state;
   private final ActionSequenceDefinition definition;
   private final List<ActionNodeExecutor<?, ?>> executorChildren = new ArrayList<>();
   private final List<ActionNodeExecutor<?, ?>> currentlyExecutingActions = new ArrayList<>();

   public ActionSequenceExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new ActionSequenceState(id, crdtInfo, saveFileDirectory));

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

      executorChildren.clear();
      currentlyExecutingActions.clear();
      updateActionSubtree(this);

      // Update concurrency ranks
      for (int i = 0; i < getState().getActionChildren().size(); i++)
      {
         ActionNodeState<?> actionChild = getState().getActionChildren().get(i);

         int concurrencyRank = 1; // Always include self

         if (i > 0 && actionChild.getDefinition().getExecuteAfterBeginning())
            ++concurrencyRank;

         if (actionChild.getExecuteAfterNode() != null && actionChild.getExecuteAfterNode().getActionIndex() < i - 1)
            ++concurrencyRank;

         int j = i + 1; // Add for any later actions executing after and prior actions
         for (; j < getState().getActionChildren().size(); j++)
         {
            ActionNodeState<?> childToCheck = getState().getActionChildren().get(j);

            if (childToCheck.getDefinition().getExecuteAfterBeginning())
               ++concurrencyRank;

            if (childToCheck.getExecuteAfterNode() != null && childToCheck.getExecuteAfterNode().getActionIndex() < i)
               ++concurrencyRank;
         }

         actionChild.setConcurrencyRank(concurrencyRank);
      }

      for (int i = 0; i < executorChildren.size(); i++)
      {
         boolean isNextForExecution = i >= state.getExecutionNextIndex() && i <= lastIndexOfConcurrentSetToExecute;
         boolean isToBeExecutedConcurrently = isNextForExecution && state.getExecutionNextIndex() != lastIndexOfConcurrentSetToExecute;

         executorChildren.get(i).getState().setIsNextForExecution(isNextForExecution);
//         executorChildren.get(i).getState().setConcurrencyRank(isToBeExecutedConcurrently);
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
            LogTools.info("End of sequence.");
            state.setAutomaticExecution(false);
         }
         else if (anyActionExecutionFailed)
         {
            LogTools.error("An action failed. Disabling automatic execution.");
            state.setAutomaticExecution(false);
         }
         else if (currentlyExecutingActions.isEmpty())
         {
            do
            {
               LogTools.info("Automatically executing action: {}", executorChildren.get(state.getExecutionNextIndex()).getClass().getSimpleName());
               executeNextAction();
            }
            while (!isEndOfSequence() && isLastExecutingActionExecuteWithNext());
         }
      }
      else if (state.pollManualExecutionRequested())
      {
         do
         {
            LogTools.info("Manually executing action: {}", executorChildren.get(state.getExecutionNextIndex()).getClass().getSimpleName());
            executeNextAction();
         }
         while (!isEndOfSequence() && isLastExecutingActionExecuteWithNext());
      }
   }

   public void updateActionSubtree(BehaviorTreeNodeExecutor<?, ?> node)
   {
      for (BehaviorTreeNodeExecutor<?, ?> child : node.getChildren())
      {
         if (child instanceof ActionNodeExecutor<?, ?> actionNode)
         {
            executorChildren.add(actionNode);
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
      ActionNodeExecutor<?, ?> actionToExecute = executorChildren.get(state.getExecutionNextIndex());

      if (actionToExecute.getState().getCanExecute())
      {
         LogTools.info("Triggering action execution: %s".formatted(actionToExecute.getDefinition().getName()));
         actionToExecute.update();
         actionToExecute.triggerActionExecution();
         actionToExecute.updateCurrentlyExecuting();
         currentlyExecutingActions.add(actionToExecute);
         state.stepForwardNextExecutionIndex();
      }
      else
      {
         LogTools.error("Cannot execute action: %s".formatted(actionToExecute.getDefinition().getName()));
         state.setAutomaticExecution(false);
      }
   }

   private boolean isLastExecutingActionExecuteWithNext()
   {
      return false;
//      return !currentlyExecutingActions.isEmpty()
//             && currentlyExecutingActions.get(currentlyExecutingActions.size() - 1).getDefinition().getExecuteAfterAction();
   }

   private boolean isEndOfSequence()
   {
      return state.getExecutionNextIndex() >= executorChildren.size();
   }

   public List<ActionNodeExecutor<?, ?>> getExecutorChildren()
   {
      return executorChildren;
   }

   public List<ActionNodeExecutor<?, ?>> getCurrentlyExecutingActions()
   {
      return currentlyExecutingActions;
   }
}

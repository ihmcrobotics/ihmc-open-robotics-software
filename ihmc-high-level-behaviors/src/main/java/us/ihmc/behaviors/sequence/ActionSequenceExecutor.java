package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
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
      int executionNextIndex = state.getExecutionNextIndex();
      if (executionNextIndex < state.getActionChildren().size())
      {
         state.getActionChildren().get(executionNextIndex).setIsNextForExecution(true);

         for (int i = executionNextIndex + 1;
              i < state.getActionChildren().size()
              && state.getActionChildren().get(i).calculateExecuteAfterActionIndex(state.getActionChildren()) < executionNextIndex; i++)
         {
            state.getActionChildren().get(i).setIsNextForExecution(true);
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
               state.getLogger().info("Automatically executing action: {}", executorChildren.get(state.getExecutionNextIndex()).getClass().getSimpleName());
               executeNextAction();
            }
         }
      }
      else if (state.pollManualExecutionRequested())
      {
         while (shouldExecuteNextAction())
         {
            state.getLogger().info("Manually executing action: {}", executorChildren.get(state.getExecutionNextIndex()).getClass().getSimpleName());
            executeNextAction();
         }
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
            actionNode.getState().setIsNextForExecution(false);
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

      ActionNodeState<?> nextNodeToExecute = executorChildren.get(state.getExecutionNextIndex()).getState();

      if (!nextNodeToExecute.getCanExecute())
      {
         state.getLogger().error("Cannot execute action: %s".formatted(nextNodeToExecute.getDefinition().getName()));
         state.setAutomaticExecution(false);
         return false;
      }

      if (state.getConcurrencyEnabled())
      {
         int executeAfterActionIndex = nextNodeToExecute.calculateExecuteAfterActionIndex(getState().getActionChildren());

         if (executeAfterActionIndex < 0) // Execute after beginning
         {
            return true;
         }
         else
         {
            return !executorChildren.get(executeAfterActionIndex).getState().getIsExecuting();
         }
      }
      else
      {
         boolean anyActionExecuting = false;
         for (ActionNodeExecutor<?, ?> executorChild : executorChildren)
         {
            anyActionExecuting |= executorChild.getState().getIsExecuting();
         }
         return  !anyActionExecuting;
      }
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

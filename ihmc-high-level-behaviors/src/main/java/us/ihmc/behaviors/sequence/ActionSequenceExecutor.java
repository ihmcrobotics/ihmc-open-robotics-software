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
   private final List<ActionNodeExecutor<?, ?>> executorChildren = new ArrayList<>();
   private final List<ActionNodeExecutor<?, ?>> currentlyExecutingActions = new ArrayList<>();
   private int lastIndexOfConcurrentSetToExecute;

   public ActionSequenceExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new ActionSequenceState(id, crdtInfo, saveFileDirectory));

      state = getState();
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

      // TODO: Go through next for execution concurrent children and set chest action's goal pelvis frames
      //   and the hand pose actions chest frames

      executorChildren.clear();
      currentlyExecutingActions.clear();
      updateActionSubtree(this);

      lastIndexOfConcurrentSetToExecute = findLastIndexOfConcurrentSetToExecute(executorChildren, getState().getExecutionNextIndex());
      for (int i = 0; i < executorChildren.size(); i++)
      {
         boolean isNextForExecution = i >= getState().getExecutionNextIndex() && i <= lastIndexOfConcurrentSetToExecute;
         boolean isToBeExecutedConcurrently = isNextForExecution && getState().getExecutionNextIndex() != lastIndexOfConcurrentSetToExecute;

         executorChildren.get(i).getState().setIsNextForExecution(isNextForExecution);
         executorChildren.get(i).getState().setIsToBeExecutedConcurrently(isToBeExecutedConcurrently);
      }

      boolean anyActionExecutionFailed = false;
      for (ActionNodeExecutor<?, ?> currentlyExecutingAction : currentlyExecutingActions)
      {
         currentlyExecutingAction.updateCurrentlyExecuting();
         anyActionExecutionFailed |= currentlyExecutingAction.getState().getFailed();
      }

      if (getState().getAutomaticExecution())
      {
         if (isEndOfSequence() || anyActionExecutionFailed)
         {
            getState().setAutomaticExecution(false);
         }
         else if (currentlyExecutingActions.isEmpty())
         {
            do
            {
               LogTools.info("Automatically executing action: {}", executorChildren.get(getState().getExecutionNextIndex()).getClass().getSimpleName());
               executeNextAction();
            }
            while (!isEndOfSequence() && getLastExecutingAction().getDefinition().getExecuteWithNextAction());
         }
      }
      else if (getState().pollManualExecutionRequested())
      {
         do
         {
            LogTools.info("Manually executing action: {}", executorChildren.get(getState().getExecutionNextIndex()).getClass().getSimpleName());
            executeNextAction();
         }
         while (!isEndOfSequence() && getLastExecutingAction().getDefinition().getExecuteWithNextAction());
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

   public static int findLastIndexOfConcurrentSetToExecute(List<ActionNodeExecutor<?, ?>> actionSequence, int executionNextIndex)
   {
      int lastIndexOfConcurrentSetToExecute = executionNextIndex;
      while (lastIndexOfConcurrentSetToExecute < actionSequence.size()
             && actionSequence.get(lastIndexOfConcurrentSetToExecute).getDefinition().getExecuteWithNextAction())
      {
         ++lastIndexOfConcurrentSetToExecute;
      }
      return lastIndexOfConcurrentSetToExecute;
   }

   private void executeNextAction()
   {
      ActionNodeExecutor<?, ?> actionToExecute = executorChildren.get(getState().getExecutionNextIndex());

      if (actionToExecute.getState().getCanExecute())
      {
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

   private ActionNodeExecutor<?, ?> getLastExecutingAction()
   {
      return currentlyExecutingActions.get(currentlyExecutingActions.size() - 1);
   }

   private boolean isEndOfSequence()
   {
      return getState().getExecutionNextIndex() >= executorChildren.size();
   }

   public List<ActionNodeExecutor<?, ?>> getExecutorChildren()
   {
      return executorChildren;
   }

   public List<ActionNodeExecutor<?, ?>> getCurrentlyExecutingActions()
   {
      return currentlyExecutingActions;
   }

   public int getLastIndexOfConcurrentSetToExecute()
   {
      return lastIndexOfConcurrentSetToExecute;
   }
}

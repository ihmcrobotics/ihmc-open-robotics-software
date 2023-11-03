package us.ihmc.behaviors.sequence;

import java.util.List;

public class BehaviorActionExecutionStatusCalculator
{
   public static void update(List<ActionNodeExecutor<?, ?>> actionSequence, int actionIndex, int executionNextIndex)
   {
      ActionNodeExecutor<?, ?> actionNode = actionSequence.get(actionIndex);

      boolean executeWithNextAction = actionNode.getDefinition().getExecuteWithNextAction();
      boolean executeWithPreviousAction;
      boolean isNextForExecution;
      if (actionIndex == 0)
      {
         executeWithPreviousAction = false;
      }
      else
      {
         executeWithPreviousAction = actionSequence.get(actionIndex - 1).getDefinition().getExecuteWithNextAction();
      }

      if (executionNextIndex > actionIndex) // If it's after, we're definitely not executing next
      {
         isNextForExecution = false;
      }
      else // We walk backwards while action have the "execute with next action" set to true
      {
         int numberOfImmediatelyPriorConcurrentActions = 0;
         int decrementingActionIndex = actionIndex - 1; // Start one action back
         while (decrementingActionIndex >= 0 && actionSequence.get(decrementingActionIndex).getDefinition().getExecuteWithNextAction())
         {
            numberOfImmediatelyPriorConcurrentActions++;
            decrementingActionIndex--;
         }

         // This also applies in the case where nothing is set to be executed concurrently
         isNextForExecution = executionNextIndex >= actionIndex - numberOfImmediatelyPriorConcurrentActions;
      }

      actionNode.getState().setActionIndex(actionIndex);
      actionNode.getState().setIsNextForExecution(isNextForExecution);
      actionNode.getState().setIsToBeExecutedConcurrently(executeWithPreviousAction || executeWithNextAction);
      actionNode.update();
   }
}

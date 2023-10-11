package us.ihmc.behaviors.sequence;

import java.util.List;

public class BehaviorActionExecutionStatusCalculator
{
   private boolean executeWithNextAction;
   private boolean executeWithPreviousAction;
   private boolean isNextForExecution;

   public <T extends BehaviorActionStateSupplier> void update(List<T> actionSequence, int actionIndex, int executionNextIndex)
   {
      executeWithNextAction = actionSequence.get(actionIndex).getState().getDefinition().getExecuteWithNextAction();
      if (actionIndex == 0)
      {
         executeWithPreviousAction = false;
      }
      else
      {
         executeWithPreviousAction = actionSequence.get(actionIndex - 1).getState().getDefinition().getExecuteWithNextAction();
      }

      if (executionNextIndex > actionIndex) // If it's after, we're definitely not executing next
      {
         isNextForExecution = false;
      }
      else // We walk backwards while action have the "execute with next action" set to true
      {
         int numberOfImmediatelyPriorConcurrentActions = 0;
         int decrementingActionIndex = actionIndex - 1; // Start one action back
         while (decrementingActionIndex >= 0 && actionSequence.get(decrementingActionIndex).getState().getDefinition().getExecuteWithNextAction())
         {
            numberOfImmediatelyPriorConcurrentActions++;
            decrementingActionIndex--;
         }

         // This also applies in the case where nothing is set to be executed concurrently
         isNextForExecution = executionNextIndex >= actionIndex - numberOfImmediatelyPriorConcurrentActions;
      }
   }

   public boolean getExecuteWithNextAction()
   {
      return executeWithNextAction;
   }

   public boolean getExecuteWithPreviousAction()
   {
      return executeWithPreviousAction;
   }

   public boolean getNextForExecution()
   {
      return isNextForExecution;
   }
}

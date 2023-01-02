package us.ihmc.behaviors.sequence;

import java.util.LinkedList;

public class BehaviorActionSequence
{
   private final LinkedList<BehaviorAction> actionSequence = new LinkedList<>();
   private boolean automaticExecution = false;
   private int excecutionNextIndex = 0;
   private BehaviorAction currentlyExecutingAction = null;

   public void update()
   {
      for (var action : actionSequence)
         action.update();

      if (automaticExecution)
      {
         boolean endOfSequence = excecutionNextIndex >= actionSequence.size();
         if (endOfSequence)
         {
            automaticExecution = false;
            currentlyExecutingAction = null;
         }
         else if (currentlyExecutingAction == null || !currentlyExecutingAction.isExecuting())
         {
            executeNextAction();
         }
      }
   }

   private void executeNextAction()
   {
      currentlyExecutingAction = actionSequence.get(excecutionNextIndex);
      currentlyExecutingAction.performAction();
      excecutionNextIndex++;
   }
}

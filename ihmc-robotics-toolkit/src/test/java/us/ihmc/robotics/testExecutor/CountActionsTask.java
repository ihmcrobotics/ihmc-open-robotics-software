package us.ihmc.robotics.testExecutor;

import us.ihmc.robotics.taskExecutor.Task;

public class CountActionsTask implements Task
{
   private int numberOfTimesTransitionIntoActionWasCalled = 0;
   private int numberOfTimesDoActionWasCalled = 0;
   private int numberOfTimesTransitionOutOfActionWasCalled = 0;

   private final int numberOfDoActionsBeforeDone;

   public CountActionsTask(int numberOfDoActionsBeforeDone)
   {
      this.numberOfDoActionsBeforeDone = numberOfDoActionsBeforeDone;
   }

   @Override
   public void doTransitionIntoAction()
   {
      numberOfTimesTransitionIntoActionWasCalled++;
   }

   @Override
   public void doAction()
   {
      numberOfTimesDoActionWasCalled++;
   }

   @Override
   public void doTransitionOutOfAction()
   {
      numberOfTimesTransitionOutOfActionWasCalled++;
   }

   @Override
   public boolean isDone()
   {
      return (numberOfTimesDoActionWasCalled >= numberOfDoActionsBeforeDone);
   }

   public boolean checkNumberOfCalls(int expectedTransitionIntoCalls, int expectedDoActionCalls, int expectedTransactionOutOfCalls)
   {
      if (this.numberOfTimesTransitionIntoActionWasCalled != expectedTransitionIntoCalls)
         return false;

      if (this.numberOfTimesDoActionWasCalled != expectedDoActionCalls)
         return false;

      if (this.numberOfTimesTransitionOutOfActionWasCalled != expectedTransactionOutOfCalls)
         return false;

      return true;

   }
}

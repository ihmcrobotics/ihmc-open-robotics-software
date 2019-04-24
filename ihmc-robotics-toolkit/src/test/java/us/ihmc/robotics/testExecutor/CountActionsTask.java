package us.ihmc.robotics.testExecutor;

import us.ihmc.robotics.stateMachine.core.State;

public class CountActionsTask implements State
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
   public void onEntry()
   {
      numberOfTimesTransitionIntoActionWasCalled++;
   }

   @Override
   public void doAction(double timeInState)
   {
      numberOfTimesDoActionWasCalled++;
   }

   @Override
   public void onExit()
   {
      numberOfTimesTransitionOutOfActionWasCalled++;
   }

   @Override
   public boolean isDone(double timeInState)
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

package us.ihmc.robotics.taskExecutor;

import us.ihmc.commons.stateMachine.core.State;

public class NullState implements State
{
   @Override
   public void onEntry()
   {
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return true;
   }
}

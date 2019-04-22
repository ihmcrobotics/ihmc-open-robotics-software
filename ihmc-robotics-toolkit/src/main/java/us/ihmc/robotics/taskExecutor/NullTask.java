package us.ihmc.robotics.taskExecutor;


public class NullTask implements Task
{
   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   @Override
   public boolean isDone()
   {
      return true;
   }
}

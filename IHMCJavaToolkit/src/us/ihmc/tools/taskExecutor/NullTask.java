package us.ihmc.tools.taskExecutor;


public class NullTask implements Task
{
   public void doTransitionIntoAction()
   {
   }

   public void doAction()
   {
   }

   public void doTransitionOutOfAction()
   {
   }

   public boolean isDone()
   {
      return true;
   }

   @Override
   public void pause()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void resume()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void stop()
   {
      // TODO Auto-generated method stub
      
   }
}

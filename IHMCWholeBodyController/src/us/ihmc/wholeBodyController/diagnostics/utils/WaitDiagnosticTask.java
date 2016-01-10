package us.ihmc.wholeBodyController.diagnostics.utils;

public class WaitDiagnosticTask extends DiagnosticTask
{
   private final double timeToWait;

   public WaitDiagnosticTask(double timeToWait)
   {
      this.timeToWait = timeToWait;
   }

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
      return getTimeInCurrentTask() > timeToWait;
   }

   @Override
   public boolean abortRequested()
   {
      return false;
   }
}

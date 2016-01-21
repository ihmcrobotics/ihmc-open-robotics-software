package us.ihmc.wholeBodyController.diagnostics.utils;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

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

   @Override
   public void attachParentYoVariableRegistry(YoVariableRegistry parentRegistry)
   {
      // No registry, do nothing.
   }

   @Override
   public String getName()
   {
      return "WaitTask";
   }
}

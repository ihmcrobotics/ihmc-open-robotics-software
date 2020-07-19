package us.ihmc.wholeBodyController.diagnostics.utils;

import java.util.ArrayDeque;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticDataReporter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class DiagnosticTask
{
   private YoDouble timeInCurrentTask;

   public abstract void doTransitionIntoAction();
   public abstract void doAction();
   public abstract void doTransitionOutOfAction();

   public abstract boolean isDone();
   public abstract boolean abortRequested();
   public abstract String getName();
   public abstract void attachParentYoVariableRegistry(YoRegistry parentRegistry);

   void setYoTimeInCurrentTask(YoDouble timeInCurrentTask)
   {
      this.timeInCurrentTask = timeInCurrentTask;
   }

   public double getTimeInCurrentTask()
   {
      return timeInCurrentTask.getDoubleValue();
   }

   public double getDesiredJointPositionOffset(OneDoFJointBasics joint)
   {
      return 0.0;
   }

   public double getDesiredJointVelocityOffset(OneDoFJointBasics joint)
   {
      return 0.0;
   }

   public double getDesiredJointTauOffset(OneDoFJointBasics joint)
   {
      return 0.0;
   }

   /**
    *  Use this method to require the high level controller to run a dataReporter.
    *  Make sure that only one thread is running at a time.
    */
   public void getDataReporterToRun(ArrayDeque<DiagnosticDataReporter> dataReportersToPack)
   {
   }
}

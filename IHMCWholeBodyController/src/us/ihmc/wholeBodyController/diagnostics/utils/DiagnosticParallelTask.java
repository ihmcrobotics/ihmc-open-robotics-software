package us.ihmc.wholeBodyController.diagnostics.utils;

import java.util.ArrayDeque;
import java.util.List;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticDataReporter;

public class DiagnosticParallelTask extends DiagnosticTask
{
   private final DiagnosticTask[] diagnosticTasks;

   public DiagnosticParallelTask(DiagnosticTask... diagnosticTasks)
   {
      this.diagnosticTasks = diagnosticTasks;
   }

   public DiagnosticParallelTask(List<? extends DiagnosticTask> diagnosticTasks)
   {
      this.diagnosticTasks = new DiagnosticTask[diagnosticTasks.size()];
      diagnosticTasks.toArray(this.diagnosticTasks);
   }

   @Override
   void setYoTimeInCurrentTask(DoubleYoVariable timeInCurrentTask)
   {
      super.setYoTimeInCurrentTask(timeInCurrentTask);

      for (int i = 0; i < diagnosticTasks.length; i++)
         diagnosticTasks[i].setYoTimeInCurrentTask(timeInCurrentTask);
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int i = 0; i < diagnosticTasks.length; i++)
         diagnosticTasks[i].doTransitionIntoAction();
   }

   @Override
   public void doAction()
   {
      for (int i = 0; i < diagnosticTasks.length; i++)
      {
         if (!diagnosticTasks[i].isDone())
            diagnosticTasks[i].doAction();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      for (int i = 0; i < diagnosticTasks.length; i++)
         diagnosticTasks[i].doTransitionOutOfAction();
   }

   @Override
   public boolean isDone()
   {
      for (int i = 0; i < diagnosticTasks.length; i++)
         if (!diagnosticTasks[i].isDone())
            return false;
      return true;
   }

   @Override
   public boolean abortRequested()
   {
      for (int i = 0; i < diagnosticTasks.length; i++)
         if (diagnosticTasks[i].abortRequested())
            return true;
      return false;
   }

   public double getDesiredJointPositionOffset(OneDoFJoint joint)
   {
      double desiredJointPositionOffset = 0.0;

      for (int i = 0; i < diagnosticTasks.length; i++)
         desiredJointPositionOffset += diagnosticTasks[i].getDesiredJointPositionOffset(joint);

      return desiredJointPositionOffset;
   }

   public double getDesiredJointVelocityOffset(OneDoFJoint joint)
   {
      double desiredJointVelocityOffset = 0.0;

      for (int i = 0; i < diagnosticTasks.length; i++)
         desiredJointVelocityOffset += diagnosticTasks[i].getDesiredJointVelocityOffset(joint);

      return desiredJointVelocityOffset;
   }

   public double getDesiredJointTauOffset(OneDoFJoint joint)
   {
      double desiredJointTauOffset = 0.0;

      for (int i = 0; i < diagnosticTasks.length; i++)
         desiredJointTauOffset += diagnosticTasks[i].getDesiredJointTauOffset(joint);

      return desiredJointTauOffset;
   }

   @Override
   public void getDataReporterToRun(ArrayDeque<DiagnosticDataReporter> dataReportersToPack)
   {
      for (int i = 0; i < diagnosticTasks.length; i++)
         diagnosticTasks[i].getDataReporterToRun(dataReportersToPack);
   }

   @Override
   public void attachParentYoVariableRegistry(YoVariableRegistry parentRegistry)
   {
      for (int i = 0; i < diagnosticTasks.length; i++)
         diagnosticTasks[i].attachParentYoVariableRegistry(parentRegistry);
   }

   @Override
   public String getName()
   {
      String ret = "{ ";
      for (int i = 0; i < diagnosticTasks.length - 1; i++)
         ret += diagnosticTasks[i].getName() + ", ";
      ret += diagnosticTasks[diagnosticTasks.length - 1].getName() + " }";
      return ret;
   }
}

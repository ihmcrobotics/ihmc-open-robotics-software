package us.ihmc.wholeBodyController.diagnostics.utils;

import java.util.ArrayDeque;
import java.util.logging.Logger;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class DiagnosticTaskExecutor
{
   private Logger logger;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoInteger currentTaskIndex;
   private final YoInteger numberOfTasksRemaining;
   private final YoBoolean isDone;
   private final YoBoolean hasAborted;
   private final YoDouble timeInCurrentTask;
   private final YoDouble switchTime;
   private final YoDouble yoTime;

   private DiagnosticTask currentTask;
   private final ArrayDeque<DiagnosticTask> taskQueue = new ArrayDeque<>();

   public DiagnosticTaskExecutor(String namePrefix, YoDouble yoTime, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      currentTaskIndex = new YoInteger(namePrefix + "CurrentTaskIndex", registry);
      numberOfTasksRemaining = new YoInteger(namePrefix + "TasksRemaining", registry);
      isDone = new YoBoolean(namePrefix + "IsDone", registry);
      hasAborted = new YoBoolean(namePrefix + "HasAborted", registry);
      timeInCurrentTask = new YoDouble(namePrefix + "TimeInCurrentTask", registry);
      switchTime = new YoDouble(namePrefix + "SwitchTime", registry);
      this.yoTime = yoTime;

      clear();
   }

   public void setupForLogging()
   {
      logger = Logger.getLogger(registry.getName());
   }

   public void submit(DiagnosticTask task)
   {
      task.attachParentYoVariableRegistry(registry);
      task.setYoTimeInCurrentTask(timeInCurrentTask);
      taskQueue.add(task);
   }

   public void doControl()
   {
      if (!isDone.getBooleanValue())
         hasAborted.set(currentTask.abortRequested());

      if (!hasAborted.getBooleanValue())
      {
         handleTransitions();
         numberOfTasksRemaining.set(taskQueue.size());
         isDone.set(isDone());
         
         if (!isDone.getBooleanValue())
         {
            timeInCurrentTask.set(yoTime.getDoubleValue() - switchTime.getDoubleValue());
            currentTask.doAction();
         }
      }
   }

   private void handleTransitions()
   {
      if (currentTask != null)
      {
         if (!currentTask.isDone())
            return;

         currentTask.doTransitionOutOfAction();

         if (logger != null)
         {
            logger.info("Diagnostic task completed.\n");
         }
      }

      if (!taskQueue.isEmpty())
      {
         currentTask = taskQueue.poll();
         currentTaskIndex.increment();
         switchTime.set(yoTime.getDoubleValue());
         currentTask.doTransitionIntoAction();
      }
      else
      {
         currentTask = null;
      }
   }

   public boolean isDone()
   {
      return taskQueue.isEmpty() && currentTask == null;
   }

   public boolean hasAborted()
   {
      return hasAborted.getBooleanValue();
   }

   public DiagnosticTask getCurrentTask()
   {
      return currentTask;
   }

   public void clear()
   {
      currentTask = null;
      taskQueue.clear();
      isDone.set(true);
      hasAborted.set(false);
      currentTaskIndex.set(0);
   }

   public void clearAllExceptCurrent()
   {
      taskQueue.clear();
      numberOfTasksRemaining.set(0);
   }
}

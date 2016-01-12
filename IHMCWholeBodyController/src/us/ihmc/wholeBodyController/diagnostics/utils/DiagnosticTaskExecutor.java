package us.ihmc.wholeBodyController.diagnostics.utils;

import java.util.ArrayDeque;
import java.util.logging.Logger;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

public class DiagnosticTaskExecutor
{
   private Logger logger;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final IntegerYoVariable currentTaskIndex;
   private final IntegerYoVariable numberOfTasksRemaining;
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable hasAborted;
   private final DoubleYoVariable timeInCurrentTask;
   private final DoubleYoVariable switchTime;
   private final DoubleYoVariable yoTime;

   private DiagnosticTask currentTask;
   private final ArrayDeque<DiagnosticTask> taskQueue = new ArrayDeque<>();

   public DiagnosticTaskExecutor(String namePrefix, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      currentTaskIndex = new IntegerYoVariable(namePrefix + "CurrentTaskIndex", registry);
      numberOfTasksRemaining = new IntegerYoVariable(namePrefix + "TasksRemaining", registry);
      isDone = new BooleanYoVariable(namePrefix + "IsDone", registry);
      hasAborted = new BooleanYoVariable(namePrefix + "HasAborted", registry);
      timeInCurrentTask = new DoubleYoVariable(namePrefix + "TimeInCurrentTask", registry);
      switchTime = new DoubleYoVariable(namePrefix + "SwitchTime", registry);
      this.yoTime = yoTime;

      clear();
   }

   public void setupForLogging()
   {
      logger = Logger.getLogger(registry.getName());
   }

   public void submit(DiagnosticTask task)
   {
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

package us.ihmc.robotics.taskExecutor;

import java.util.ArrayDeque;

public class TaskExecutor
{
   private boolean printDebugStatements;
   private Task currentTask;
   private final NullTask nullTask = new NullTask();
   private final ArrayDeque<Task> taskQueue = new ArrayDeque<Task>();

   public TaskExecutor()
   {
      clear();
   }

   public void submit(Task task)
   {
      taskQueue.add(task);
   }

   public void doControl()
   {
      handleTransitions();
      currentTask.doAction();
   }

   public void handleTransitions()
   {
      if (currentTask.isDone())
      {
         currentTask.doTransitionOutOfAction();

         if (!taskQueue.isEmpty())
         {
            currentTask = taskQueue.poll();
            if (printDebugStatements)
               System.out.println("+++ " + getClass().getSimpleName() + ": transitioning into new task:\n" + currentTask.toString());
         }
         else
         {
            currentTask = nullTask;
         }

         currentTask.doTransitionIntoAction();
      }
   }

   public boolean isDone()
   {
      return ((currentTask == nullTask) && taskQueue.isEmpty());
   }

   public Task getCurrentTask()
   {
      return currentTask;
   }

   public Task getLastTask()
   {
      return taskQueue.peekLast();
   }

   public Task getNextTask()
   {
      return taskQueue.peek();
   }

   protected ArrayDeque<Task> getTaskQueue()
   {
      return taskQueue;
   }

   public void clear()
   {
      currentTask = nullTask;
      taskQueue.clear();
   }

   public void clearAllExceptCurrent()
   {
      taskQueue.clear();
   }

   public void setPrintDebugStatements(boolean printDebugStatements)
   {
      this.printDebugStatements = printDebugStatements;
   }
}

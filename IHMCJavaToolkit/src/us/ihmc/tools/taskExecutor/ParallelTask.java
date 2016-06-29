package us.ihmc.tools.taskExecutor;

import java.util.LinkedHashMap;
import java.util.Map;

public class ParallelTask<T> implements Task
{
   // TODO convert to ObjectObjectMap<T, TaskExecutor>
   private final Map<T, TaskExecutor> executors = new LinkedHashMap<T, TaskExecutor>();

   public void submit(T executorKey, Task task)
   {
      TaskExecutor executor = executors.get(executorKey);
      if (executor == null)
      {
         executor = new TaskExecutor();
         executors.put(executorKey, executor);
      }

      executor.submit(task);
   }

   public void clear(T executorKey)
   {
      TaskExecutor taskExecutor = executors.get(executorKey);
      if (taskExecutor != null)
         taskExecutor.clear();
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (TaskExecutor taskExecutor : executors.values())
      {
         taskExecutor.handleTransitions();
      }
   }

   @Override
   public void doAction()
   {
      for (TaskExecutor taskExecutor : executors.values())
      {
         taskExecutor.doControl();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // empty
   }

   @Override
   public boolean isDone()
   {
      for (TaskExecutor taskExecutor : executors.values())
      {
         if (!taskExecutor.isDone())
            return false;
      }

      return true;
   }
}

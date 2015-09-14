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

   public void doTransitionIntoAction()
   {
      for (TaskExecutor taskExecutor : executors.values())
      {
         taskExecutor.handleTransitions();
      }
   }

   public void doAction()
   {
      for (TaskExecutor taskExecutor : executors.values())
      {
         taskExecutor.doControl();
      }
   }

   public void doTransitionOutOfAction()
   {
      // empty
   }

   public boolean isDone()
   {
      for (TaskExecutor taskExecutor : executors.values())
      {
         if (!taskExecutor.isDone())
            return false;
      }

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

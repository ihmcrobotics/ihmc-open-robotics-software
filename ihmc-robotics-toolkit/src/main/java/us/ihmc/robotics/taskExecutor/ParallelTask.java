package us.ihmc.robotics.taskExecutor;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

public class ParallelTask<T> implements Task
{
   // TODO convert to ObjectObjectMap<T, TaskExecutor>
   private final Map<T, TaskExecutor> executorMap = new LinkedHashMap<T, TaskExecutor>();
   private final ArrayList<TaskExecutor> executors = new ArrayList<>();

   public void submit(T executorKey, Task task)
   {
      TaskExecutor executor = executorMap.get(executorKey);
      if (executor == null)
      {
         executor = new TaskExecutor();
         executorMap.put(executorKey, executor);
         executors.add(executor);
      }

      executor.submit(task);
   }

   public void clear(T executorKey)
   {
      TaskExecutor taskExecutor = executorMap.get(executorKey);
      if (taskExecutor != null)
         taskExecutor.clear();
      
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int i = 0; i < executors.size(); i++)
      {
         executors.get(i).handleTransitions();
      }
   }

   @Override
   public void doAction()
   {
      for (int i = 0; i < executors.size(); i++)
      {
         executors.get(i).doControl();
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
      for (int i = 0; i < executors.size(); i++)
      {
         
         if (!executors.get(i).isDone())
            return false;
      }

      return true;
   }

   public void clearAll()
   {
      for (int i = 0; i < executors.size(); i++)
      {
         executors.get(i).clear();
      }
   }
}

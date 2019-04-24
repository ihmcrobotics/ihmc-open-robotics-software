package us.ihmc.robotics.taskExecutor;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachineClock;
import us.ihmc.yoVariables.variable.YoDouble;

public class ParallelTask<T> implements State
{
   // TODO convert to ObjectObjectMap<T, TaskExecutor>
   private final Map<T, TaskExecutor> executorMap = new LinkedHashMap<T, TaskExecutor>();
   private final ArrayList<TaskExecutor> executors = new ArrayList<>();
   private final StateMachineClock clock;

   public ParallelTask(YoDouble yotime)
   {
      clock = StateMachineClock.clock(yotime);
   }
   
   //the new constructor with yoTime should be used.
   @Deprecated
   public ParallelTask()
   {
      clock = StateMachineClock.dummyClock();
   }
   
   public void submit(T executorKey, State task)
   {
      TaskExecutor executor = executorMap.get(executorKey);
      if (executor == null)
      {
         executor = new TaskExecutor(clock);
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
   public void onEntry()
   {
      for (int i = 0; i < executors.size(); i++)
      {
         executors.get(i).handleTransitions();
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      for (int i = 0; i < executors.size(); i++)
      {
         executors.get(i).doControl();
      }
   }

   @Override
   public void onExit()
   {
      // empty
   }

   @Override
   public boolean isDone(double timeInState)
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

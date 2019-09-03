package us.ihmc.robotics.taskExecutor;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachineClock;
import us.ihmc.yoVariables.variable.YoDouble;

public class ParallelState<T> implements State
{
   // TODO convert to ObjectObjectMap<T, TaskExecutor>
   private final Map<T, StateExecutor> executorMap = new LinkedHashMap<T, StateExecutor>();
   private final ArrayList<StateExecutor> executors = new ArrayList<>();
   private final StateMachineClock clock;

   public ParallelState(YoDouble yotime)
   {
      clock = StateMachineClock.clock(yotime);
   }
   
   //the new constructor with yoTime should be used.
   @Deprecated
   public ParallelState()
   {
      clock = StateMachineClock.dummyClock();
   }
   
   public void submit(T executorKey, State state)
   {
      StateExecutor executor = executorMap.get(executorKey);
      if (executor == null)
      {
         executor = new StateExecutor(clock);
         executorMap.put(executorKey, executor);
         executors.add(executor);
      }

      executor.submit(state);
   }

   public void clear(T executorKey)
   {
      StateExecutor stateExecutor = executorMap.get(executorKey);
      if (stateExecutor != null)
         stateExecutor.clear();
      
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

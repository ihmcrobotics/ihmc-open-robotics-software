package us.ihmc.robotics.taskExecutor;

import java.util.ArrayDeque;

import us.ihmc.commons.stateMachine.core.State;
import us.ihmc.commons.stateMachine.core.StateMachineClock;

public class StateExecutor
{
   private boolean printDebugStatements;
   private State currentTask;
   private final NullState nullTask = new NullState();
   private final ArrayDeque<State> taskQueue = new ArrayDeque<State>();
   private final StateMachineClock clock;

   public StateExecutor(StateMachineClock clock)
   {
      this.clock = clock;
      clear();
   }
   //a statemachine clock should be passed in slowly removing this constructor
   @Deprecated
   public StateExecutor()
   {
      clock = StateMachineClock.dummyClock();
      clear();
   }

   public void submit(State task)
   {
      taskQueue.add(task);
   }

   public void doControl()
   {
      handleTransitions();
      currentTask.doAction(clock.getTimeInCurrentState());
   }

   public void handleTransitions()
   {
      if (currentTask.isDone(clock.getTimeInCurrentState()))
      {
         currentTask.onExit(clock.getTimeInCurrentState());

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

         currentTask.onEntry();
         clock.notifyStateChanged();
      }
   }

   public boolean isDone()
   {
      return ((currentTask == nullTask) && taskQueue.isEmpty());
   }

   public State getCurrentTask()
   {
      return currentTask;
   }

   public State getLastTask()
   {
      return taskQueue.peekLast();
   }

   public State getNextTask()
   {
      return taskQueue.peek();
   }

   protected ArrayDeque<State> getTaskQueue()
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
   
   protected StateMachineClock getClock()
   {
      return clock;
   }
}

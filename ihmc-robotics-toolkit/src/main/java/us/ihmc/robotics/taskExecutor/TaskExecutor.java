package us.ihmc.robotics.taskExecutor;

import java.util.ArrayDeque;

import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachineClock;

public class TaskExecutor
{
   private boolean printDebugStatements;
   private State currentTask;
   private final NullTask nullTask = new NullTask();
   private final ArrayDeque<State> taskQueue = new ArrayDeque<State>();
   private final StateMachineClock clock;

   public TaskExecutor(StateMachineClock clock)
   {
      this.clock = clock;
      clear();
   }
   //a statemachine clock should be passed in slowly removing this constructor
   @Deprecated
   public TaskExecutor()
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
         currentTask.onExit();

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
}

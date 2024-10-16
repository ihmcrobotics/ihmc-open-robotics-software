package us.ihmc.robotics.testExecutor;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertNull;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.stateMachine.core.State;
import us.ihmc.robotics.taskExecutor.NullState;
import us.ihmc.robotics.taskExecutor.StateExecutor;

public class TaskExecutorTest
{

   @Test
   public void testEmptyExecutor()
   {
      StateExecutor taskExecutor = new StateExecutor();

      assertTrue(taskExecutor.isDone());
      State currentTask = taskExecutor.getCurrentTask();

      assertTrue(currentTask.isDone(Double.NaN));

      State nextTask = taskExecutor.getNextTask();
      assertNull(nextTask);

      State lastTask = taskExecutor.getLastTask();
      assertNull(lastTask);
   }

   @Test
   public void testWithOneNullTask()
   {
      StateExecutor taskExecutor = new StateExecutor();
      assertTrue(taskExecutor.isDone());

      NullState nullTask0 = new NullState();
      taskExecutor.submit(nullTask0);

      assertFalse(taskExecutor.isDone());
      assertTrue(nullTask0 == taskExecutor.getNextTask());
      assertTrue(nullTask0 == taskExecutor.getLastTask());

      taskExecutor.doControl();
      assertTrue(nullTask0 == taskExecutor.getCurrentTask());
      assertNull(taskExecutor.getNextTask());
      assertNull(taskExecutor.getLastTask());
      assertFalse(taskExecutor.isDone());

      taskExecutor.doControl();
      assertTrue(taskExecutor.getCurrentTask() instanceof NullState);
      assertNull(taskExecutor.getNextTask());
      assertNull(taskExecutor.getLastTask());
      assertTrue(taskExecutor.isDone());
   }

   @Test
   public void testWithSeveralNullTasks()
   {
      StateExecutor taskExecutor = new StateExecutor();

      NullState nullTask0 = new NullState();
      NullState nullTask1 = new NullState();
      NullState nullTask2 = new NullState();
      NullState nullTask3 = new NullState();
      NullState nullTask4 = new NullState();
      taskExecutor.submit(nullTask0);
      taskExecutor.submit(nullTask1);
      taskExecutor.submit(nullTask2);
      taskExecutor.submit(nullTask3);
      taskExecutor.submit(nullTask4);

      assertFalse(taskExecutor.isDone());
      assertTrue(taskExecutor.getCurrentTask() instanceof NullState);
      assertTrue(nullTask0 == taskExecutor.getNextTask());
      assertTrue(nullTask4 == taskExecutor.getLastTask());

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(nullTask0 == taskExecutor.getCurrentTask());
      assertTrue(nullTask1 == taskExecutor.getNextTask());
      assertTrue(nullTask4 == taskExecutor.getLastTask());

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(nullTask1 == taskExecutor.getCurrentTask());
      assertTrue(nullTask2 == taskExecutor.getNextTask());
      assertTrue(nullTask4 == taskExecutor.getLastTask());

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(nullTask2 == taskExecutor.getCurrentTask());
      assertTrue(nullTask3 == taskExecutor.getNextTask());
      assertTrue(nullTask4 == taskExecutor.getLastTask());

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(nullTask3 == taskExecutor.getCurrentTask());
      assertTrue(nullTask4 == taskExecutor.getNextTask());
      assertTrue(nullTask4 == taskExecutor.getLastTask());

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(nullTask4 == taskExecutor.getCurrentTask());
      assertNull(taskExecutor.getNextTask());
      assertNull(taskExecutor.getLastTask());

      taskExecutor.doControl();
      assertTrue(taskExecutor.isDone());
      assertFalse(nullTask4 == taskExecutor.getCurrentTask());
      assertNull(taskExecutor.getNextTask());
      assertNull(taskExecutor.getLastTask());

      assertTrue(taskExecutor.getCurrentTask() instanceof NullState);
   }

   @Test
   public void testSomeTasks()
   {
      int[] doActionsPerTask = new int[] {1};
      runATest(doActionsPerTask);

      doActionsPerTask = new int[] {1, 2};

      runATest(doActionsPerTask);

      doActionsPerTask = new int[] {2, 3, 7, 10, 1, 3};

      runATest(doActionsPerTask);

      doActionsPerTask = new int[] {2, 3, 7, 10, 1, 1};

      runATest(doActionsPerTask);

   }

   @Test
   public void testAddingTasksOnTheFly()
   {
      StateExecutor taskExecutor = new StateExecutor();

      CountActionsTask exampleTask0 = new CountActionsTask(2);

      taskExecutor.submit(exampleTask0);

      assertFalse(taskExecutor.isDone());
      assertTrue(exampleTask0.checkNumberOfCalls(0, 0, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(exampleTask0.checkNumberOfCalls(1, 1, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(exampleTask0.checkNumberOfCalls(1, 2, 0));

      taskExecutor.doControl();
      assertTrue(taskExecutor.isDone());
      assertTrue(exampleTask0.checkNumberOfCalls(1, 2, 1));

      CountActionsTask exampleTask1 = new CountActionsTask(2);
      taskExecutor.submit(exampleTask1);

      assertFalse(taskExecutor.isDone());
      assertTrue(exampleTask0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTask1.checkNumberOfCalls(0, 0, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(exampleTask0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTask1.checkNumberOfCalls(1, 1, 0));

      CountActionsTask exampleTask2 = new CountActionsTask(3);
      taskExecutor.submit(exampleTask2);

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(exampleTask0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTask1.checkNumberOfCalls(1, 2, 0));
      assertTrue(exampleTask2.checkNumberOfCalls(0, 0, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(exampleTask0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTask1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTask2.checkNumberOfCalls(1, 1, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(exampleTask0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTask1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTask2.checkNumberOfCalls(1, 2, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(exampleTask0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTask1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTask2.checkNumberOfCalls(1, 3, 0));

      taskExecutor.doControl();
      assertTrue(taskExecutor.isDone());
      assertTrue(exampleTask0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTask1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTask2.checkNumberOfCalls(1, 3, 1));
   }

   private void runATest(int[] doActionsPerTask)
   {
      StateExecutor taskExecutor = new StateExecutor();

      CountActionsTask[] exampleTasks = new CountActionsTask[doActionsPerTask.length];

      for (int i = 0; i < doActionsPerTask.length; i++)
      {
         CountActionsTask exampleTask = new CountActionsTask(doActionsPerTask[i]);
         exampleTasks[i] = exampleTask;

         taskExecutor.submit(exampleTask);
         assertEquals(exampleTask, taskExecutor.getLastTask());
      }

      int currentTaskIndex = -1;
      int currentTaskDoActionCount = 0;

      while (!taskExecutor.isDone())
      {
         taskExecutor.doControl();
         currentTaskDoActionCount++;

         if ((currentTaskIndex == -1) || (currentTaskDoActionCount > doActionsPerTask[currentTaskIndex])) // Finished the previous task.
         {
            currentTaskIndex++;
            currentTaskDoActionCount = 1;
         }

         if (!taskExecutor.isDone())
         {
            assertEquals(exampleTasks[currentTaskIndex], taskExecutor.getCurrentTask());
            assertTrue(exampleTasks[currentTaskIndex].checkNumberOfCalls(1, currentTaskDoActionCount, 0));
         }

         if (currentTaskIndex < exampleTasks.length - 1)
         {
            assertEquals(exampleTasks[currentTaskIndex + 1], taskExecutor.getNextTask());
         }

         State lastTask = taskExecutor.getLastTask();
         if (lastTask != null)
            assertEquals(exampleTasks[exampleTasks.length - 1], lastTask);

         for (int i = 0; i < exampleTasks.length; i++)
         {
            if (i < currentTaskIndex)
            {
               assertTrue(exampleTasks[i].isDone(Double.NaN));
               assertTrue(exampleTasks[i].checkNumberOfCalls(1, doActionsPerTask[i], 1));

            }
            else if (currentTaskDoActionCount < doActionsPerTask[currentTaskIndex])
               assertFalse(exampleTasks[i].isDone(Double.NaN));
         }

         if (currentTaskIndex < doActionsPerTask.length)
         {
            if ((currentTaskDoActionCount >= doActionsPerTask[currentTaskIndex]))
            {
               exampleTasks[currentTaskIndex].checkNumberOfCalls(1, currentTaskDoActionCount, 1);
               assertTrue(exampleTasks[currentTaskIndex].isDone(Double.NaN));
            }
            else
            {
               exampleTasks[currentTaskIndex].checkNumberOfCalls(1, currentTaskDoActionCount, 0);
               assertFalse(exampleTasks[currentTaskIndex].isDone(Double.NaN));
            }
         }
      }

      assertEquals(exampleTasks.length, currentTaskIndex);
   }

}

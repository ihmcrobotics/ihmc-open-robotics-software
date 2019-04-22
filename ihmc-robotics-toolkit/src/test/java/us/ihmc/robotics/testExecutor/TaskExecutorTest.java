package us.ihmc.robotics.testExecutor;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertNull;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.Test;

import us.ihmc.robotics.taskExecutor.NullTask;
import us.ihmc.robotics.taskExecutor.Task;
import us.ihmc.robotics.taskExecutor.TaskExecutor;
public class TaskExecutorTest
{

   @Test
   public void testEmptyExecutor()
   {
      TaskExecutor taskExecutor = new TaskExecutor();

      assertTrue(taskExecutor.isDone());
      Task currentTask = taskExecutor.getCurrentTask();

      assertTrue(currentTask.isDone());

      Task nextTask = taskExecutor.getNextTask();
      assertNull(nextTask);

      Task lastTask = taskExecutor.getLastTask();
      assertNull(lastTask);
   }

   @Test
   public void testWithOneNullTask()
   {
      TaskExecutor taskExecutor = new TaskExecutor();
      assertTrue(taskExecutor.isDone());

      NullTask nullTask0 = new NullTask();
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
      assertTrue(taskExecutor.getCurrentTask() instanceof NullTask);
      assertNull(taskExecutor.getNextTask());
      assertNull(taskExecutor.getLastTask());
      assertTrue(taskExecutor.isDone());
   }

   @Test
   public void testWithSeveralNullTasks()
   {
      TaskExecutor taskExecutor = new TaskExecutor();

      NullTask nullTask0 = new NullTask();
      NullTask nullTask1 = new NullTask();
      NullTask nullTask2 = new NullTask();
      NullTask nullTask3 = new NullTask();
      NullTask nullTask4 = new NullTask();
      taskExecutor.submit(nullTask0);
      taskExecutor.submit(nullTask1);
      taskExecutor.submit(nullTask2);
      taskExecutor.submit(nullTask3);
      taskExecutor.submit(nullTask4);

      assertFalse(taskExecutor.isDone());
      assertTrue(taskExecutor.getCurrentTask() instanceof NullTask);
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

      assertTrue(taskExecutor.getCurrentTask() instanceof NullTask);
   }

   @Test
   public void testSomeTasks()
   {
      int[] doActionsPerTask = new int[] { 1 };
      runATest(doActionsPerTask);

      doActionsPerTask = new int[] { 1, 2 };

      runATest(doActionsPerTask);

      doActionsPerTask = new int[] { 2, 3, 7, 10, 1, 3 };

      runATest(doActionsPerTask);

      doActionsPerTask = new int[] { 2, 3, 7, 10, 1, 1 };

      runATest(doActionsPerTask);

   }

   @Test
   public void testAddingTasksOnTheFly()
   {
      TaskExecutor taskExecutor = new TaskExecutor();

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
      TaskExecutor taskExecutor = new TaskExecutor();

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

         Task lastTask = taskExecutor.getLastTask();
         if (lastTask != null)
            assertEquals(exampleTasks[exampleTasks.length - 1], lastTask);

         for (int i = 0; i < exampleTasks.length; i++)
         {
            if (i < currentTaskIndex)
            {
               assertTrue(exampleTasks[i].isDone());
               assertTrue(exampleTasks[i].checkNumberOfCalls(1, doActionsPerTask[i], 1));

            }
            else if (currentTaskDoActionCount < doActionsPerTask[currentTaskIndex])
               assertFalse(exampleTasks[i].isDone());
         }

         if (currentTaskIndex < doActionsPerTask.length)
         {
            if ((currentTaskDoActionCount >= doActionsPerTask[currentTaskIndex]))
            {
               exampleTasks[currentTaskIndex].checkNumberOfCalls(1, currentTaskDoActionCount, 1);
               assertTrue(exampleTasks[currentTaskIndex].isDone());
            }
            else
            {
               exampleTasks[currentTaskIndex].checkNumberOfCalls(1, currentTaskDoActionCount, 0);
               assertFalse(exampleTasks[currentTaskIndex].isDone());
            }
         }
      }

      assertEquals(exampleTasks.length, currentTaskIndex);
   }

}

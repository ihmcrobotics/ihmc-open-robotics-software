package us.ihmc.robotics.testExecutor;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.Test;

import us.ihmc.robotics.stateMachine.core.StateMachineClock;
import us.ihmc.robotics.taskExecutor.NullState;
import us.ihmc.robotics.taskExecutor.ParallelState;
import us.ihmc.robotics.taskExecutor.StateExecutor;
import us.ihmc.yoVariables.variable.YoDouble;

public class ParallelTaskTest
{

   @Test
   public void testOneSerialTask()
   {

      StateExecutor taskExecutor = new StateExecutor();
      ParallelState<ParallelTaskKey> parallelTask = new ParallelState<ParallelTaskKey>();

      ParallelTaskKey taskKeyA = new ParallelTaskKey();

      CountActionsTask exampleTaskA0 = new CountActionsTask(2);
      parallelTask.submit(taskKeyA, exampleTaskA0);

      taskExecutor.submit(parallelTask);

      assertFalse(taskExecutor.isDone());

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 1, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 2, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 2, 1));

      taskExecutor.doControl();
      assertTrue(taskExecutor.isDone());
      assertTrue(parallelTask != taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 2, 1));

   }

   @Test
   public void testTwoSerialTasks()
   {
      StateExecutor taskExecutor = new StateExecutor();
      ParallelState<ParallelTaskKey> parallelTask = new ParallelState<ParallelTaskKey>();

      ParallelTaskKey taskKeyA = new ParallelTaskKey();

      CountActionsTask exampleTaskA0 = new CountActionsTask(2);
      CountActionsTask exampleTaskA1 = new CountActionsTask(2);
      parallelTask.submit(taskKeyA, exampleTaskA0);
      parallelTask.submit(taskKeyA, exampleTaskA1);

      taskExecutor.submit(parallelTask);

      assertFalse(taskExecutor.isDone());

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 1, 0));
      assertTrue(exampleTaskA1.checkNumberOfCalls(0, 0, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 2, 0));
      assertTrue(exampleTaskA1.checkNumberOfCalls(0, 0, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskA1.checkNumberOfCalls(1, 1, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskA1.checkNumberOfCalls(1, 2, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskA1.checkNumberOfCalls(1, 2, 1));

      taskExecutor.doControl();
      assertTrue(parallelTask != taskExecutor.getCurrentTask());
      assertTrue(taskExecutor.isDone());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskA1.checkNumberOfCalls(1, 2, 1));

   }

   @Test
   public void testTwoParallelTasks()
   {
      StateExecutor taskExecutor = new StateExecutor();
      ParallelState<ParallelTaskKey> parallelTask = new ParallelState<ParallelTaskKey>();

      ParallelTaskKey taskKeyA = new ParallelTaskKey();
      ParallelTaskKey taskKeyB = new ParallelTaskKey();

      CountActionsTask exampleTaskA0 = new CountActionsTask(2);
      CountActionsTask exampleTaskB0 = new CountActionsTask(3);
      parallelTask.submit(taskKeyA, exampleTaskA0);
      parallelTask.submit(taskKeyB, exampleTaskB0);

      taskExecutor.submit(parallelTask);

      assertFalse(taskExecutor.isDone());

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 1, 0));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 1, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 2, 0));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 2, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 3, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 3, 1));

      taskExecutor.doControl();
      assertTrue(taskExecutor.isDone());
      assertTrue(parallelTask != taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 3, 1));
   }

   @Test
   public void testABunchOfParallelAndSeriesTasks()
   {
      StateExecutor taskExecutor = new StateExecutor();
      ParallelState<ParallelTaskKey> parallelTask = new ParallelState<ParallelTaskKey>();

      ParallelTaskKey taskKeyA = new ParallelTaskKey();
      ParallelTaskKey taskKeyB = new ParallelTaskKey();
      ParallelTaskKey taskKeyC = new ParallelTaskKey();

      CountActionsTask exampleTaskA0 = new CountActionsTask(1);
      CountActionsTask exampleTaskA1 = new CountActionsTask(2);
      CountActionsTask exampleTaskB0 = new CountActionsTask(2);
      CountActionsTask exampleTaskB1 = new CountActionsTask(2);
      CountActionsTask exampleTaskC0 = new CountActionsTask(5);
      parallelTask.submit(taskKeyA, exampleTaskA0);
      parallelTask.submit(taskKeyA, exampleTaskA1);
      parallelTask.submit(taskKeyB, exampleTaskB0);
      parallelTask.submit(taskKeyB, exampleTaskB1);
      parallelTask.submit(taskKeyC, exampleTaskC0);

      taskExecutor.submit(parallelTask);

      assertFalse(taskExecutor.isDone());

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 1, 0));
      assertTrue(exampleTaskA1.checkNumberOfCalls(0, 0, 0));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 1, 0));
      assertTrue(exampleTaskB1.checkNumberOfCalls(0, 0, 0));
      assertTrue(exampleTaskC0.checkNumberOfCalls(1, 1, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 1, 1));
      assertTrue(exampleTaskA1.checkNumberOfCalls(1, 1, 0));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 2, 0));
      assertTrue(exampleTaskB1.checkNumberOfCalls(0, 0, 0));
      assertTrue(exampleTaskC0.checkNumberOfCalls(1, 2, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 1, 1));
      assertTrue(exampleTaskA1.checkNumberOfCalls(1, 2, 0));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB1.checkNumberOfCalls(1, 1, 0));
      assertTrue(exampleTaskC0.checkNumberOfCalls(1, 3, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 1, 1));
      assertTrue(exampleTaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB1.checkNumberOfCalls(1, 2, 0));
      assertTrue(exampleTaskC0.checkNumberOfCalls(1, 4, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 1, 1));
      assertTrue(exampleTaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskC0.checkNumberOfCalls(1, 5, 0));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 1, 1));
      assertTrue(exampleTaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskC0.checkNumberOfCalls(1, 5, 1));

      CountActionsTask exampleTaskA2 = new CountActionsTask(2);
      parallelTask.submit(taskKeyA, exampleTaskA2);

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 1, 1));
      assertTrue(exampleTaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskA2.checkNumberOfCalls(1, 1, 0));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskC0.checkNumberOfCalls(1, 5, 1));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 1, 1));
      assertTrue(exampleTaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskA2.checkNumberOfCalls(1, 2, 0));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskC0.checkNumberOfCalls(1, 5, 1));

      taskExecutor.doControl();
      assertFalse(taskExecutor.isDone());
      assertTrue(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 1, 1));
      assertTrue(exampleTaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskA2.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskC0.checkNumberOfCalls(1, 5, 1));

      taskExecutor.doControl();
      assertTrue(taskExecutor.isDone());
      assertFalse(parallelTask == taskExecutor.getCurrentTask());
      assertTrue(taskExecutor.getCurrentTask() instanceof NullState);
      assertTrue(exampleTaskA0.checkNumberOfCalls(1, 1, 1));
      assertTrue(exampleTaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskA2.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskB1.checkNumberOfCalls(1, 2, 1));
      assertTrue(exampleTaskC0.checkNumberOfCalls(1, 5, 1));
   }

   private class ParallelTaskKey
   {

   }

}

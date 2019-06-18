package us.ihmc.robotics.testExecutor;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.robotics.taskExecutor.ParallelState;
import us.ihmc.robotics.taskExecutor.PipeLine;
public class PipeLineTest
{

   @Test
   public void testEmptyPipeline()
   {
      PipeLine<ParallelTaskKey> pipeLine = new PipeLine<ParallelTaskKey>();

      pipeLine.doControl();
      pipeLine.doControl();
      assertTrue(pipeLine.isDone());
   }

   @Test
   public void testPipelineWithOneSingleTaskStage()
   {
      PipeLine<ParallelTaskKey> pipeLine = new PipeLine<ParallelTaskKey>();
      CountActionsTask stage0SingleTask = new CountActionsTask(2);

      pipeLine.submitSingleTaskStage(stage0SingleTask);

      assertFalse(pipeLine.isDone());
      pipeLine.doControl();
      assertTrue(stage0SingleTask.checkNumberOfCalls(1, 1, 0));
      assertFalse(pipeLine.isDone());
      pipeLine.doControl();
      assertTrue(stage0SingleTask.checkNumberOfCalls(1, 2, 0));
      assertFalse(pipeLine.isDone());
      pipeLine.doControl();
      assertTrue(stage0SingleTask.checkNumberOfCalls(1, 2, 1));
      assertTrue(pipeLine.isDone());
   }

   @Test
   public void testPipelineWithTwoSingleTaskStages()
   {
      PipeLine<ParallelTaskKey> pipeLine = new PipeLine<ParallelTaskKey>();
      CountActionsTask stage0SingleTask = new CountActionsTask(2);
      pipeLine.submitSingleTaskStage(stage0SingleTask);
      CountActionsTask stage1SingleTask = new CountActionsTask(3);
      pipeLine.submitSingleTaskStage(stage1SingleTask);

      pipeLine.doControl();
      assertTrue(stage0SingleTask.checkNumberOfCalls(1, 1, 0));
      assertTrue(stage1SingleTask.checkNumberOfCalls(0, 0, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0SingleTask.checkNumberOfCalls(1, 2, 0));
      assertTrue(stage1SingleTask.checkNumberOfCalls(0, 0, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(pipeLine.getCurrentStage() == stage1SingleTask);
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 1, 0));
      assertTrue(stage0SingleTask.checkNumberOfCalls(1, 2, 1));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 2, 0));
      assertTrue(stage0SingleTask.checkNumberOfCalls(1, 2, 1));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 3, 0));
      assertTrue(stage0SingleTask.checkNumberOfCalls(1, 2, 1));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 3, 1));
      assertTrue(stage0SingleTask.checkNumberOfCalls(1, 2, 1));
      assertTrue(pipeLine.isDone());
   }

   @Test
   public void testPipelineOneStageTwoPipesThenOneSingleTaskStage()
   {
      PipeLine<ParallelTaskKey> pipeLine = new PipeLine<ParallelTaskKey>();

      ParallelTaskKey taskKeyA = new ParallelTaskKey();
      ParallelTaskKey taskKeyB = new ParallelTaskKey();

      CountActionsTask stage0TaskA = new CountActionsTask(1);
      CountActionsTask stage0TaskB = new CountActionsTask(2);
      CountActionsTask stage1SingleTask = new CountActionsTask(2);

      pipeLine.submitTaskForPallelPipesStage(taskKeyA, stage0TaskA);
      pipeLine.submitTaskForPallelPipesStage(taskKeyB, stage0TaskB);
      pipeLine.submitSingleTaskStage(stage1SingleTask);

      pipeLine.doControl();
      assertTrue(pipeLine.getCurrentStage() instanceof ParallelState);
      assertTrue(stage0TaskA.checkNumberOfCalls(1, 1, 0));
      assertTrue(stage0TaskB.checkNumberOfCalls(1, 1, 0));
      assertTrue(stage1SingleTask.checkNumberOfCalls(0, 0, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(pipeLine.getCurrentStage() instanceof ParallelState);
      assertTrue(stage0TaskA.checkNumberOfCalls(1, 1, 1));
      assertTrue(stage0TaskB.checkNumberOfCalls(1, 2, 0));
      assertTrue(stage1SingleTask.checkNumberOfCalls(0, 0, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(pipeLine.getCurrentStage() instanceof ParallelState);
      assertTrue(stage0TaskA.checkNumberOfCalls(1, 1, 1));
      assertTrue(stage0TaskB.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(0, 0, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(pipeLine.getCurrentStage() == stage1SingleTask);
      assertTrue(stage0TaskA.checkNumberOfCalls(1, 1, 1));
      assertTrue(stage0TaskB.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 1, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA.checkNumberOfCalls(1, 1, 1));
      assertTrue(stage0TaskB.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 2, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA.checkNumberOfCalls(1, 1, 1));
      assertTrue(stage0TaskB.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 2, 1));
      assertTrue(pipeLine.isDone());
   }

   @Test
   public void testTwoParallelPipes()
   {
      PipeLine<ParallelTaskKey> pipeLine = new PipeLine<ParallelTaskKey>();

      ParallelTaskKey taskKeyA = new ParallelTaskKey();
      ParallelTaskKey taskKeyB = new ParallelTaskKey();

      CountActionsTask stage0TaskA0 = new CountActionsTask(2);
      CountActionsTask stage0TaskA1 = new CountActionsTask(2);
      CountActionsTask stage0TaskB0 = new CountActionsTask(2);

      pipeLine.submitTaskForPallelPipesStage(taskKeyA, stage0TaskA0);
      pipeLine.submitTaskForPallelPipesStage(taskKeyA, stage0TaskA1);
      pipeLine.submitTaskForPallelPipesStage(taskKeyB, stage0TaskB0);

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 1, 0));
      assertTrue(stage0TaskA1.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 1, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 0));
      assertTrue(stage0TaskA1.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 1, 0));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 2, 0));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(pipeLine.isDone());

   }

   @Test
   public void testThreePipesWithParallelAndSeriesTasks()
   {
      PipeLine<ParallelTaskKey> pipeLine = new PipeLine<ParallelTaskKey>();

      ParallelTaskKey taskKeyA = new ParallelTaskKey();
      ParallelTaskKey taskKeyB = new ParallelTaskKey();
      ParallelTaskKey taskKeyC = new ParallelTaskKey();

      CountActionsTask stage0TaskA0 = new CountActionsTask(2);
      CountActionsTask stage0TaskA1 = new CountActionsTask(2);
      CountActionsTask stage0TaskB0 = new CountActionsTask(2);

      CountActionsTask stage1SingleTask = new CountActionsTask(1);

      CountActionsTask stage2TaskA0 = new CountActionsTask(2);
      CountActionsTask stage2TaskA1 = new CountActionsTask(2);
      CountActionsTask stage2TaskB0 = new CountActionsTask(2);
      CountActionsTask stage2TaskC0 = new CountActionsTask(5);

      pipeLine.submitTaskForPallelPipesStage(taskKeyA, stage0TaskA0);
      pipeLine.submitTaskForPallelPipesStage(taskKeyA, stage0TaskA1);
      pipeLine.submitTaskForPallelPipesStage(taskKeyB, stage0TaskB0);

      pipeLine.submitSingleTaskStage(stage1SingleTask);

      pipeLine.submitTaskForPallelPipesStage(taskKeyA, stage2TaskA0);
      pipeLine.submitTaskForPallelPipesStage(taskKeyA, stage2TaskA1);
      pipeLine.submitTaskForPallelPipesStage(taskKeyB, stage2TaskB0);
      pipeLine.submitTaskForPallelPipesStage(taskKeyC, stage2TaskC0);

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 1, 0));
      assertTrue(stage0TaskA1.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 1, 0));
      assertTrue(stage1SingleTask.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskA0.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskA1.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskB0.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskC0.checkNumberOfCalls(0, 0, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 0));
      assertTrue(stage0TaskA1.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 0));
      assertTrue(stage1SingleTask.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskA0.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskA1.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskB0.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskC0.checkNumberOfCalls(0, 0, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 1, 0));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskA0.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskA1.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskB0.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskC0.checkNumberOfCalls(0, 0, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 2, 0));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskA0.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskA1.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskB0.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskC0.checkNumberOfCalls(0, 0, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskA0.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskA1.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskB0.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskC0.checkNumberOfCalls(0, 0, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 1, 0));
      assertTrue(stage2TaskA0.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskA1.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskB0.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskC0.checkNumberOfCalls(0, 0, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 1, 1));
      assertTrue(stage2TaskA0.checkNumberOfCalls(1, 1, 0));
      assertTrue(stage2TaskA1.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskB0.checkNumberOfCalls(1, 1, 0));
      assertTrue(stage2TaskC0.checkNumberOfCalls(1, 1, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 1, 1));
      assertTrue(stage2TaskA0.checkNumberOfCalls(1, 2, 0));
      assertTrue(stage2TaskA1.checkNumberOfCalls(0, 0, 0));
      assertTrue(stage2TaskB0.checkNumberOfCalls(1, 2, 0));
      assertTrue(stage2TaskC0.checkNumberOfCalls(1, 2, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 1, 1));
      assertTrue(stage2TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage2TaskA1.checkNumberOfCalls(1, 1, 0));
      assertTrue(stage2TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage2TaskC0.checkNumberOfCalls(1, 3, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 1, 1));
      assertTrue(stage2TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage2TaskA1.checkNumberOfCalls(1, 2, 0));
      assertTrue(stage2TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage2TaskC0.checkNumberOfCalls(1, 4, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 1, 1));
      assertTrue(stage2TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage2TaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage2TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage2TaskC0.checkNumberOfCalls(1, 5, 0));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 1, 1));
      assertTrue(stage2TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage2TaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage2TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage2TaskC0.checkNumberOfCalls(1, 5, 1));
      assertFalse(pipeLine.isDone());

      pipeLine.doControl();
      assertTrue(stage0TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage0TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage1SingleTask.checkNumberOfCalls(1, 1, 1));
      assertTrue(stage2TaskA0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage2TaskA1.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage2TaskB0.checkNumberOfCalls(1, 2, 1));
      assertTrue(stage2TaskC0.checkNumberOfCalls(1, 5, 1));
      assertTrue(pipeLine.isDone());
   }

   private class ParallelTaskKey
   {

   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForPackage(PipeLineTest.class);
   }
}

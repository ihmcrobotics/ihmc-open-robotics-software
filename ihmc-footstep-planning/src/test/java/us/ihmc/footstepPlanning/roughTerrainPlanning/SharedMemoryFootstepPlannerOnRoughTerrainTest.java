package us.ihmc.footstepPlanning.roughTerrainPlanning;

import org.junit.Test;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments;
import us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.Assert.assertTrue;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.*;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI.*;

public abstract class SharedMemoryFootstepPlannerOnRoughTerrainTest
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected JavaFXMessager messager;
   protected boolean keepUIUp = false;

   public abstract FootstepPlannerType getPlannerType();


   @ContinuousIntegrationTest(estimatedDuration = 20)
   @Test(timeout = 30000000)
   public void test()
   {
      List<String> testsToRun = getTestsToRun();
      for (String test : testsToRun)
      {
         PrintTools.info("Running test " + test);
         runTestAndAssert(getTestData(test));
         PrintTools.info("Test " + test + " passed.");
         ThreadTools.sleep(100);
      }
   }



   private void runTestAndAssert(PlannerTestEnvironments.PlannerTestData testData)
   {
      submitInfoToUI(testData);

      ThreadTools.sleep(10);

      AtomicReference<Boolean> receivedPlan = new AtomicReference<>(false);
      AtomicReference<Boolean> receivedResult = new AtomicReference<>(false);
      messager.registerTopicListener(FootstepPlanTopic, request -> receivedPlan.set(true));
      messager.registerTopicListener(PlanningResultTopic, request -> receivedResult.set(true));
      AtomicReference<FootstepPlan> footstepPlanReference = messager.createInput(FootstepPlanTopic);
      AtomicReference<FootstepPlanningResult> footstepPlanningResult = messager.createInput(PlanningResultTopic);

      messager.submitMessage(ComputePathTopic, true);

      double timeout = 5.0;
      double totalTimeTaken = 0.0;
      long sleepDuration = 10;
      while (!receivedPlan.get() && !receivedResult.get())
      {
         if (totalTimeTaken > timeout + 5.0)
            throw new RuntimeException("Waited too long for a result.");

         ThreadTools.sleep(sleepDuration);
         totalTimeTaken += Conversions.millisecondsToSeconds(sleepDuration);
      }

      ThreadTools.sleep(10);

      assertTrue("Planning result is invalid, result was " + footstepPlanningResult.get(), footstepPlanningResult.get().validForExecution());
      assertTrue(PlannerTools.isGoalNextToLastStep(testData.getGoalPose(), footstepPlanReference.get()));

      if (keepUIUp)
         ThreadTools.sleepForever();
   }

   private void submitInfoToUI(PlannerTestEnvironments.PlannerTestData testData)
   {
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlanarRegionDataTopic, testData.getPlanarRegionsList());
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.GoalPositionTopic, testData.getGoalPosition());
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.GoalOrientationTopic, testData.getGoalOrientation());
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.StartPositionTopic, testData.getStartPosition());
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.StartOrientationTopic, testData.getStartOrientation());

      messager.submitMessage(PlannerTypeTopic, getPlannerType());
   }
















   public List<String> getTestsToRun()
   {
      List<String> tests = new ArrayList<>();
      tests.add(staircase);
      tests.add(wall);
      tests.add(overCinderBlockField);
      tests.add(steppingStones);
      tests.add(stepUpsAndDownsScoringDifficult);
      tests.add(stepAfterPitchDown);
      tests.add(stepAfterPitchUp);
      tests.add(compareStepBeforeGap);
      tests.add(simpleStepOnBox);
      tests.add(simpleStepOnBoxTwo);
      tests.add(random);
      tests.add(simpleGaps);
//      tests.add(partialGaps);
      tests.add(box);
//      tests.add(spiralStaircase);
      tests.add(hole);

      return tests;
   }



}

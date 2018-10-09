package us.ihmc.footstepPlanning.roughTerrainPlanning;

import org.junit.After;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments;
import us.ihmc.footstepPlanning.testTools.PlanningTest;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;

import java.util.concurrent.atomic.AtomicReference;

import static org.junit.Assert.assertTrue;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.*;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI.ComputePathTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI.PlannerParametersTopic;

public abstract class FootstepPlannerOnRoughTerrainTest implements PlanningTest
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   protected JavaFXMessager messager;
   protected AtomicReference<FootstepPlannerParameters> parametersReference;

   public abstract boolean assertPlannerReturnedResult();

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testOnStaircase()
   {
      // run the test
      runTestAndAssert(getTestData(staircase));
   }

   @ContinuousIntegrationTest(estimatedDuration = 2.5)
   @Test(timeout = 30000)
   public void testWithWall()
   {
      // run the test
      runTestAndAssert(getTestData(wall));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testOverCinderBlockField()
   {
      // run the test
      runTestAndAssert(getTestData(overCinderBlockField));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 3000000)
   public void testSteppingStones()
   {
      // run the test
      runTestAndAssert(getTestData(steppingStones));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testStepUpsAndDownsScoringDifficult()
   {
      runTestAndAssert(getTestData(stepUpsAndDownsScoringDifficult));
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testStepAfterPitchedUp()
   {
      runTestAndAssert(getTestData(stepAfterPitchUp));
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testStepAfterPitchedDown()
   {
      runTestAndAssert(getTestData(stepAfterPitchDown));
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testCompareStepBeforeGap()
   {
      runTestAndAssert(getTestData(compareStepBeforeGap));
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testSimpleStepOnBox()
   {
      runTestAndAssert(getTestData(simpleStepOnBox));
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testSimpleStepOnBoxTwo()
   {
      runTestAndAssert(getTestData(simpleStepOnBoxTwo));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testRandomEnvironment()
   {
      runTestAndAssert(getTestData(random));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testSimpleGaps()
   {
      runTestAndAssert(getTestData(simpleGaps));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testPartialGaps()
   {
      runTestAndAssert(getTestData(partialGaps));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testWalkingAroundBox()
   {
      // run the test
      runTestAndAssert(getTestData(box));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testSpiralStaircase()
   {
      // run the test
      runTestAndAssert(getTestData(spiralStaircase));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testWalkingAroundHole()
   {
      // run the test
      runTestAndAssert(getTestData(hole));
   }

   protected FootstepPlannerParameters getDefaultPlannerParameters()
   {
      return new DefaultFootstepPlanningParameters();
   }

   protected FootstepPlannerParameters getPlannerParameters()
   {
      if (parametersReference == null)
         return getDefaultPlannerParameters();

      return parametersReference.get();
   }

   private void runTestAndAssert(PlannerTestEnvironments.PlannerTestData testData)
   {
      FootstepPlan footstepPlan = PlannerTools
            .runPlanner(getPlanner(), testData.getStartPose(), testData.getStartSide(), testData.getGoalPose(), testData.getPlanarRegionsList(),
                        assertPlannerReturnedResult());

      if (assertPlannerReturnedResult())
         assertTrue(PlannerTools.isGoalNextToLastStep(testData.getGoalPose(), footstepPlan));

      if (messager != null && visualize())
      {
         parametersReference = messager.createInput(PlannerParametersTopic, getDefaultPlannerParameters());

         submitInfoToUI(testData, footstepPlan);

         ThreadTools.sleep(10);

         messager.registerTopicListener(ComputePathTopic, request -> iterateOnPlan(testData));

         if (keepUp())
            ThreadTools.sleepForever();
      }
   }

   private void iterateOnPlan(PlannerTestEnvironments.PlannerTestData testData)
   {
      PrintTools.info("Iterating");
      FootstepPlan footstepPlan = PlannerTools
            .runPlanner(getPlanner(), testData.getStartPose(), testData.getStartSide(), testData.getGoalPose(), testData.getPlanarRegionsList(),
                        assertPlannerReturnedResult());

      submitInfoToUI(testData, footstepPlan);
   }

   private void submitInfoToUI(PlannerTestEnvironments.PlannerTestData testData, FootstepPlan footstepPlan)
   {
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlanarRegionDataTopic, testData.getPlanarRegionsList());
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.GoalPositionTopic, testData.getGoalPosition());
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.GoalOrientationTopic, testData.getGoalOrientation());
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.StartPositionTopic, testData.getStartPosition());
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.StartOrientationTopic, testData.getStartOrientation());

      messager.submitMessage(FootstepPlannerSharedMemoryAPI.FootstepPlanTopic, footstepPlan);
   }

}

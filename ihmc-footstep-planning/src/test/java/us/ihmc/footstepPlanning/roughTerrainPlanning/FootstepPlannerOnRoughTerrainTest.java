package us.ihmc.footstepPlanning.roughTerrainPlanning;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerStatus;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments;
import us.ihmc.footstepPlanning.testTools.PlanningTest;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;

import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.robotics.Assert.*;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ComputePathTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.FootstepPlanTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerParametersTopic;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.*;

public abstract class FootstepPlannerOnRoughTerrainTest implements PlanningTest
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private FootstepPlannerUI ui;

   protected static boolean visualize = true;
   protected JavaFXMessager messager;

   private boolean checkForBodyBoxCollision = false;
   private double bodyBoxDepth = 0.3;
   private double bodyBoxWidth = 0.7;
   private double bodyBoxOffsetX = 0.0;

   @Before
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      checkForBodyBoxCollision = false;

      if (visualize)
      {
         ApplicationRunner.runApplication(new Application()
         {
            @Override
            public void start(Stage stage) throws Exception
            {
               messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
               messager.startMessager();

               ui = FootstepPlannerUI.createMessagerUI(stage, messager);
               ui.show();
            }

            @Override
            public void stop()
            {
               ui.stop();
               Platform.exit();
            }
         });

         double maxStartUpTime = 5.0;
         double currentTime = 0.0;
         long sleepDuration = 100;
         while (ui == null)
         {
            if (currentTime > maxStartUpTime)
               throw new RuntimeException("Failed to start UI");

            currentTime += Conversions.millisecondsToSeconds(sleepDuration);
            ThreadTools.sleep(sleepDuration);
         }
      }

      setupInternal();
   }

   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();

      if (ui != null)
      {
         ui.stop();
         Platform.exit();
      }

      destroyInternal();
   }

   public void setCheckForBodyBoxCollision(boolean checkForBodyBoxCollision)
   {
      this.checkForBodyBoxCollision = checkForBodyBoxCollision;
   }

   public void setBodyBoxDepth(double bodyBoxDepth)
   {
      this.bodyBoxDepth = bodyBoxDepth;
   }

   public void setBodyBoxWidth(double bodyBoxWidth)
   {
      this.bodyBoxWidth = bodyBoxWidth;
   }

   public void setBodyBoxOffsetX(double bodyBoxOffsetX)
   {
      this.bodyBoxOffsetX = bodyBoxOffsetX;
   }

   protected AtomicReference<FootstepPlannerParameters> parametersReference = new AtomicReference<>(getDefaultPlannerParameters());

   protected abstract void setupInternal();

   protected abstract void destroyInternal();

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

   public void testDownCorridor()
   {
      runTestAndAssert(getTestData(corridor));
   }

   public void testBetweenTwoBollards()
   {
      runTestAndAssert(getTestData(bollards));
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
      return new DefaultFootstepPlanningParameters()

      {
         @Override
         public double getBodyBoxBaseX()
         {
            return bodyBoxOffsetX;
         }

         @Override
         public double getBodyBoxDepth()
         {
            return bodyBoxDepth;
         }

         @Override
         public double getBodyBoxWidth()
         {
            return bodyBoxWidth;
         }

         @Override
         public boolean checkForBodyBoxCollisions()
         {
            return checkForBodyBoxCollision;
         }
      };
   }

   protected FootstepPlannerParameters getPlannerParameters()
   {
      if (parametersReference == null)
         return getDefaultPlannerParameters();

      return parametersReference.get();
   }

   private void runTestAndAssert(PlannerTestEnvironments.PlannerTestData testData)
   {
      if (messager != null && visualize())
         submitInfoToUI(testData);

      Random random = new Random(324);
      testData.getPlanarRegionsList().getPlanarRegionsAsList().forEach(region -> region.setRegionId(random.nextInt()));

      FootstepPlanner planner = getPlanner();
      FootstepPlan footstepPlan = PlannerTools
            .runPlanner(planner, testData.getStartPose(), testData.getStartSide(), testData.getGoalPose(), testData.getPlanarRegionsList(),
                        assertPlannerReturnedResult());

      if (assertPlannerReturnedResult())
         assertTrue(PlannerTools.isGoalNextToLastStep(testData.getGoalPose(), footstepPlan));

      if (messager != null && visualize())
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.PlannerStatusTopic, FootstepPlannerStatus.PLANNING_STEPS);
         messager.registerTopicListener(PlannerParametersTopic, message ->
         {
            // TODO set parameters from message
         });

         messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanTopic, footstepPlan);
         messager.submitMessage(FootstepPlannerMessagerAPI.PlannerStatusTopic, FootstepPlannerStatus.IDLE);
         messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeTakenTopic, planner.getPlanningDuration());

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

      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanTopic, footstepPlan);
   }

   private void submitInfoToUI(PlannerTestEnvironments.PlannerTestData testData)
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionDataTopic, testData.getPlanarRegionsList());
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalPositionTopic, testData.getGoalPosition());
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalOrientationTopic, testData.getGoalOrientation());
      messager.submitMessage(FootstepPlannerMessagerAPI.StartPositionTopic, testData.getStartPosition());
      messager.submitMessage(FootstepPlannerMessagerAPI.StartOrientationTopic, testData.getStartOrientation());
   }
}

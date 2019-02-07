package us.ihmc.footstepPlanning.roughTerrainPlanning;

import static us.ihmc.robotics.Assert.*;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ComputePathTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.FootstepPlanTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerTypeTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlanningResultTopic;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.box;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.compareStepBeforeGap;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.getTestData;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.hole;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.overCinderBlockField;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.random;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.simpleGaps;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.simpleStepOnBox;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.simpleStepOnBoxTwo;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.staircase;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.stepAfterPitchDown;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.stepAfterPitchUp;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.stepUpsAndDownsScoringDifficult;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.steppingStones;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.wall;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.components.FootstepPathCalculatorModule;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.SharedMemoryMessager;

public abstract class MessagerFootstepPlannerOnRoughTerrainTest
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected SharedMemoryMessager messager = null;
   protected FootstepPathCalculatorModule module = null;
   protected FootstepPlannerUI ui = null;
   protected boolean keepUIUp = false;

   public abstract FootstepPlannerType getPlannerType();

   private static boolean visualize = false;

   @BeforeEach
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (visualize)
         messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
      else
         messager = new SharedMemoryMessager(FootstepPlannerMessagerAPI.API);

      try
      {
         messager.startMessager();
      }
      catch (Exception e)
      {
         throw new RuntimeException("Failed to start message.");
      }

      module = FootstepPathCalculatorModule.createMessagerModule(messager);
      module.start();

      if (visualize)
      {

         ApplicationRunner.runApplication(new Application()
         {
            @Override
            public void start(Stage stage) throws Exception
            {
               ui = FootstepPlannerUI.createMessagerUI(stage, (SharedMemoryJavaFXMessager) messager);
               ui.show();
            }

            @Override
            public void stop() throws Exception
            {
               ui.stop();
               Platform.exit();
            }
         });

         double maxWaitTime = 5.0;
         double totalTime = 0.0;
         long sleepDuration = 100;

         while (ui == null)
         {
            if (totalTime > maxWaitTime)
               throw new RuntimeException("Timed out waiting for the UI to start.");
            ThreadTools.sleep(sleepDuration);
            totalTime += Conversions.millisecondsToSeconds(sleepDuration);
         }

      }

   }

   @AfterEach
   public void tearDown() throws Exception
   {
      module.stop();
      messager.closeMessager();
      if (ui != null)
         ui.stop();

      module = null;
      messager = null;
      ui = null;
   }

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
      while (!receivedPlan.get() || !receivedResult.get() || footstepPlanReference.get() == null)
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
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionDataTopic, testData.getPlanarRegionsList());
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalPositionTopic, testData.getGoalPosition());
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalOrientationTopic, testData.getGoalOrientation());
      messager.submitMessage(FootstepPlannerMessagerAPI.StartPositionTopic, testData.getStartPosition());
      messager.submitMessage(FootstepPlannerMessagerAPI.StartOrientationTopic, testData.getStartOrientation());

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

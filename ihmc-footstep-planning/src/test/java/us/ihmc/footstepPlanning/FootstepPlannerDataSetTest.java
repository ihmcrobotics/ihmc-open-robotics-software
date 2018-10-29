package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.*;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.apache.commons.lang3.tuple.Pair;
import org.junit.After;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.tools.FootstepPlannerDataExporter;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.components.FootstepPathCalculatorModule;
import us.ihmc.javaFXToolkit.messager.Messager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryMessager;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.FootstepPlanTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerTypeTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlanningResultTopic;

public abstract class FootstepPlannerDataSetTest
{
   private static final double bambooTimeScaling = 4.0;

   // Whether to start the UI or not.
   protected static boolean VISUALIZE = false;
   // For enabling helpful prints.
   private static boolean DEBUG = false;
   private static boolean VERBOSE = false;

   private FootstepPlannerUI ui = null;
   private Messager messager = null;

   private final AtomicReference<FootstepPlan> plannerPlanReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> plannerResultReference = new AtomicReference<>(null);
   private final AtomicReference<Boolean> plannerReceivedPlan = new AtomicReference<>(false);
   private final AtomicReference<Boolean> plannerReceivedResult = new AtomicReference<>(false);

   private AtomicReference<FootstepPlan> uiFootstepPlanReference;
   private AtomicReference<FootstepPlanningResult> uiPlanningResultReference;
   private final AtomicReference<Boolean> uiReceivedPlan = new AtomicReference<>(false);
   private final AtomicReference<Boolean> uiReceivedResult = new AtomicReference<>(false);

   private final AtomicReference<FootstepPlan> expectedPlan = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlan> actualPlan = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> expectedResult = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> actualResult = new AtomicReference<>(null);

   private FootstepPathCalculatorModule module = null;

   protected abstract FootstepPlannerType getPlannerType();

   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (VISUALIZE)
         messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
      else
         messager = new SharedMemoryMessager(FootstepPlannerMessagerAPI.API);

      module = new FootstepPathCalculatorModule(messager);
      module.start();

      try
      {
         messager.startMessager();
      }
      catch (Exception e)
      {
         throw new RuntimeException("Failed to start messager.");
      }

      if (VISUALIZE)
      {
         createUI(messager);
      }

      ThreadTools.sleep(1000);

      messager.registerTopicListener(FootstepPlanTopic, request -> uiReceivedPlan.set(true));
      messager.registerTopicListener(PlanningResultTopic, request -> uiReceivedResult.set(true));

      uiFootstepPlanReference = messager.createInput(FootstepPlanTopic);
      uiPlanningResultReference = messager.createInput(PlanningResultTopic);
   }

   @After
   public void tearDown() throws Exception
   {
      module.stop();
      messager.closeMessager();
      if (ui != null)
         ui.stop();
      ui = null;

      uiFootstepPlanReference = null;
      uiPlanningResultReference = null;

      module = null;
      messager = null;
   }

   private void resetAllAtomics()
   {
      plannerPlanReference.set(null);
      plannerResultReference.set(null);
      plannerReceivedPlan.set(false);
      plannerReceivedResult.set(false);

      if (uiFootstepPlanReference != null)
         uiFootstepPlanReference.set(null);
      if (uiPlanningResultReference != null)
         uiPlanningResultReference.set(null);
      uiReceivedPlan.set(false);
      uiReceivedResult.set(false);

      expectedPlan.set(null);
      actualPlan.set(null);
      expectedResult.set(null);
      actualResult.set(null);
   }

   @Test(timeout = 500000)
   @ContinuousIntegrationTest(estimatedDuration = 13.0)
   public void testDatasetsWithoutOcclusion()
   {
      List<FootstepPlannerUnitTestDataset> allDatasets = FootstepPlannerIOTools
            .loadAllFootstepPlannerDatasetsWithoutOcclusions(FootstepPlannerDataExporter.class);
      runAssertionsOnAllDatasets(this::runAssertions, allDatasets);
   }

   @Test(timeout = 500000)
   @ContinuousIntegrationTest(estimatedDuration = 13.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   public void testDatasetsWithoutOcclusionInDevelopment()
   {
      List<FootstepPlannerUnitTestDataset> allDatasets = FootstepPlannerIOTools
            .loadAllFootstepPlannerDatasetsWithoutOcclusionsInDevelopment(FootstepPlannerDataExporter.class);
      runAssertionsOnAllDatasets(this::runAssertions, allDatasets);
   }

   protected void runAssertionsOnDataset(DatasetTestRunner datasetTestRunner, String datasetName)
   {
      FootstepPlannerUnitTestDataset dataset = FootstepPlannerIOTools.loadDataset(FootstepPlannerDataExporter.class, datasetName);

      resetAllAtomics();
      String errorMessages = datasetTestRunner.testDataset(dataset);
      Assert.assertTrue("Errors:" + errorMessages, errorMessages.isEmpty());
   }

   protected void runAssertionsOnAllDatasets(DatasetTestRunner datasetTestRunner, List<FootstepPlannerUnitTestDataset> allDatasets)
   {
      if (VERBOSE || DEBUG)
         PrintTools.info("Unit test files found: " + allDatasets.size());

      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      int numberOfFailingTests = 0;
      for (int i = 0; i < allDatasets.size(); i++)
      {
         FootstepPlannerUnitTestDataset dataset = allDatasets.get(i);
         if (DEBUG || VERBOSE)
            PrintTools.info("Testing file: " + dataset.getDatasetName());

         if(!dataset.getTypes().contains(getPlannerType()))
         {
            if(DEBUG || VERBOSE)
               PrintTools.info(dataset.getDatasetName() + " does not contain planner type " + getPlannerType() + ", skipping");
            continue;
         }

         resetAllAtomics();
         String errorMessagesForCurrentFile = datasetTestRunner.testDataset(dataset);
         if (!errorMessagesForCurrentFile.isEmpty())
            numberOfFailingTests++;

         if (DEBUG || VERBOSE)
         {
            String result = errorMessagesForCurrentFile.isEmpty() ? "passed" : "failed";
            PrintTools.info(dataset.getDatasetName() + " " + result);
         }

         ThreadTools.sleep(500); // Apparently need to give some time for the prints to appear in the right order.
      }

      String message = "Number of failing datasets: " + numberOfFailingTests + " out of " + allDatasets.size();
      if (VISUALIZE)
      {
         PrintTools.info(message);
         ThreadTools.sleepForever();
      }
      else
      {
         Assert.assertEquals(message, numberOfFailingTests, 0);
      }
   }

   public String runAssertions(FootstepPlannerUnitTestDataset dataset)
   {
      ThreadTools.sleep(1000);
      packPlanningRequest(dataset, messager);
      return findPlanAndAssertGoodResult(dataset);
   }

   protected void packPlanningRequest(FootstepPlannerUnitTestDataset dataset, Messager messager)
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.StartPositionTopic, dataset.getStart());
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalPositionTopic, dataset.getGoal());
      messager.submitMessage(PlannerTypeTopic, getPlannerType());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionDataTopic, dataset.getPlanarRegionsList());

      double timeMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeoutTopic, timeMultiplier * dataset.getTimeout(getPlannerType()));

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerHorizonLengthTopic, Double.MAX_VALUE);

      if (dataset.hasGoalOrientation())
         messager.submitMessage(FootstepPlannerMessagerAPI.GoalOrientationTopic, dataset.getGoalOrientation());
      if (dataset.hasStartOrientation())
         messager.submitMessage(FootstepPlannerMessagerAPI.StartOrientationTopic, dataset.getStartOrientation());

      messager.submitMessage(FootstepPlannerMessagerAPI.ComputePathTopic, true);

      if (DEBUG)
         PrintTools.info("Sending out planning request packet.");
   }

   protected String assertPlanIsValid(String datasetName, FootstepPlanningResult result, FootstepPlan plan, Point3D goal)
   {
      String errorMessage = "";

      errorMessage += assertTrue(datasetName, "Planning result for " + datasetName + " is invalid, result was " + result, result.validForExecution());

      if (result.validForExecution())
      {
         errorMessage += assertTrue(datasetName,
                                    datasetName + " did not reach goal. Made it to " + PlannerTools.getEndPosition(plan) + ", trying to get to " + goal,
                                    PlannerTools.isGoalNextToLastStep(goal, plan));
      }

      return errorMessage;
   }

   private String assertTrue(String datasetName, String message, boolean condition)
   {
      if (VISUALIZE || DEBUG)
      {
         if (!condition)
            PrintTools.error(datasetName + ": " + message);
      }
      return !condition ? "\n" + message : "";
   }

   protected void createUI(Messager messager)
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

   protected double totalTimeTaken;

   protected String waitForResult(ConditionChecker conditionChecker, double maxTimeToWait, String prefix)
   {
      String errorMessage = "";
      long waitTime = 10;
      while (conditionChecker.checkCondition())
      {
         if (totalTimeTaken > maxTimeToWait)
         {
            errorMessage += prefix + " timed out waiting for a result.\n";
            return errorMessage;
         }

         ThreadTools.sleep(waitTime);
         totalTimeTaken += Conversions.millisecondsToSeconds(waitTime);
         queryUIResults();
         queryPlannerResults();
      }

      return errorMessage;
   }

   protected String validateResult(ConditionChecker conditionChecker, FootstepPlanningResult result, String prefix)
   {
      String errorMessage = "";

      if (!conditionChecker.checkCondition())
      {
         errorMessage += prefix + " failed to find a valid result. Result : " + result + "\n";
      }

      return errorMessage;
   }

   protected String waitForPlan(ConditionChecker conditionChecker, double maxTimeToWait, String prefix)
   {
      String errorMessage = "";

      while (conditionChecker.checkCondition())
      {
         long waitTime = 10;

         if (totalTimeTaken > maxTimeToWait)
         {
            errorMessage += prefix + " timed out waiting on plan.\n";
            return errorMessage;
         }

         ThreadTools.sleep(waitTime);
         totalTimeTaken += Conversions.millisecondsToSeconds(waitTime);
         queryUIResults();
         queryPlannerResults();
      }

      return errorMessage;
   }

   protected void queryUIResults()
   {
      if (uiReceivedPlan.get() && uiFootstepPlanReference.get() != null && actualPlan.get() == null)
      {
         if (DEBUG)
            PrintTools.info("Received a plan from the UI.");
         actualPlan.set(uiFootstepPlanReference.getAndSet(null));
         uiReceivedPlan.set(false);
      }

      if (uiReceivedResult.get() && uiPlanningResultReference.get() != null)
      {
         if (DEBUG)
            PrintTools.info("Received a result " + uiPlanningResultReference.get() + " from the UI.");
         actualResult.set(uiPlanningResultReference.getAndSet(null));
         uiReceivedResult.set(false);
      }
   }

   protected void queryPlannerResults()
   {
      if (plannerReceivedPlan.get() && plannerPlanReference.get() != null && expectedPlan.get() == null)
      {
         if (DEBUG)
            PrintTools.info("Received a plan from the planner.");
         expectedPlan.set(plannerPlanReference.getAndSet(null));
         plannerReceivedPlan.set(false);
      }

      if (plannerReceivedResult.get() && plannerResultReference.get() != null)
      {
         if (DEBUG)
            PrintTools.info("Received a result " + plannerResultReference.get() + " from the planner.");
         expectedResult.set(plannerResultReference.getAndSet(null));
         plannerReceivedResult.set(false);
      }
   }

   public String findPlanAndAssertGoodResult(FootstepPlannerUnitTestDataset dataset)
   {
      totalTimeTaken = 0.0;
      double timeoutMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double maxTimeToWait = 2.0 * timeoutMultiplier * dataset.getTimeout(getPlannerType());
      String datasetName = dataset.getDatasetName();

      queryUIResults();

      String errorMessage = "";

      errorMessage += waitForResult(() -> actualResult.get() == null, maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      errorMessage += validateResult(() -> actualResult.get().validForExecution(), actualResult.get(), datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      errorMessage += waitForPlan(() -> actualPlan.get() == null, maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      FootstepPlanningResult result = actualResult.getAndSet(null);
      FootstepPlan plan = actualPlan.getAndSet(null);

      uiReceivedPlan.getAndSet(false);
      uiReceivedResult.getAndSet(false);

      errorMessage += assertPlanIsValid(datasetName, result, plan, dataset.getGoal());

      ThreadTools.sleep(1000);
      return errorMessage;
   }

   protected static interface DatasetTestRunner
   {
      String testDataset(FootstepPlannerUnitTestDataset dataset);
   }

   protected static interface ConditionChecker
   {
      boolean checkCondition();
   }
}

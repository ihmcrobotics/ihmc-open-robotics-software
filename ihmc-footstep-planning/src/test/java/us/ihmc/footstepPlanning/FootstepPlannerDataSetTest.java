package us.ihmc.footstepPlanning;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.FootstepPlanTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerTypeTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlanningResultTopic;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.AfterEach;
import us.ihmc.robotics.Assert;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.tools.FootstepPlannerDataExporter;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.components.FootstepPathCalculatorModule;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;

public abstract class FootstepPlannerDataSetTest
{
   protected static final double bambooTimeScaling = 4.0;

   // Whether to start the UI or not.
   protected static boolean VISUALIZE = true;
   // For enabling helpful prints.
   protected static boolean DEBUG = true;
   protected static boolean VERBOSE = true;

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

   @BeforeEach
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

   @AfterEach
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

   @Test
   public void testDatasetsWithoutOcclusion()
   {
      List<FootstepPlannerUnitTestDataset> allDatasets = FootstepPlannerIOTools
            .loadAllFootstepPlannerDatasetsWithoutOcclusions(FootstepPlannerDataExporter.class);
      runAssertionsOnAllDatasets(this::runAssertions, allDatasets);
   }

   @Test
   @Disabled
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

   private void runAssertionsOnAllDatasets(DatasetTestRunner datasetTestRunner, List<FootstepPlannerUnitTestDataset> allDatasets)
   {
      if (VERBOSE || DEBUG)
         LogTools.info("Unit test files found: " + allDatasets.size());

      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      int numberOfFailingTests = 0;
      List<String> failingDatasets = new ArrayList<>();
      int numbberOfTestedSets = 0;
      for (int i = 0; i < allDatasets.size(); i++)
      {
         FootstepPlannerUnitTestDataset dataset = allDatasets.get(i);
         if (DEBUG || VERBOSE)
            LogTools.info("Testing file: " + dataset.getDatasetName());

         if(!dataset.getTypes().contains(getPlannerType()))
         {
            if(DEBUG || VERBOSE)
               LogTools.info(dataset.getDatasetName() + " does not contain planner type " + getPlannerType() + ", skipping");
            continue;
         }

         numbberOfTestedSets++;
         resetAllAtomics();
         String errorMessagesForCurrentFile = datasetTestRunner.testDataset(dataset);
         if (!errorMessagesForCurrentFile.isEmpty())
         {
            numberOfFailingTests++;
            failingDatasets.add(dataset.getDatasetName());
         }

         if (DEBUG || VERBOSE)
         {
            String result = errorMessagesForCurrentFile.isEmpty() ? "passed" : "failed";
            LogTools.info(dataset.getDatasetName() + " " + result);
         }

         ThreadTools.sleep(500); // Apparently need to give some time for the prints to appear in the right order.
      }

      String message = "Number of failing datasets: " + numberOfFailingTests + " out of " + numbberOfTestedSets;
      message += "\n Datasets failing: ";
      for (int i = 0; i < failingDatasets.size(); i++)
      {
         message += "\n" + failingDatasets.get(i);
      }
      if (VISUALIZE)
      {
         LogTools.info(message);
         ThreadTools.sleepForever();
      }
      else
      {
         Assert.assertEquals(message, 0, numberOfFailingTests);
      }
   }

   protected String runAssertions(FootstepPlannerUnitTestDataset dataset)
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
         LogTools.info("Sending out planning request packet.");
   }



   protected String assertPlanIsValid(String datasetName, FootstepPlanningResult result, FootstepPlan plan, Point3D goal)
   {
      String errorMessage = "";

      if(!result.validForExecution())
      {
         errorMessage = "Planning result for " + datasetName + " is invalid, result was " + result;
      }
      else if(!PlannerTools.isGoalNextToLastStep(goal, plan))
      {
         errorMessage = datasetName + " did not reach goal. Made it to " + PlannerTools.getEndPosition(plan) + ", trying to get to " + goal;
      }

      if((VISUALIZE || DEBUG) && !errorMessage.isEmpty())
         LogTools.error(errorMessage);

      return errorMessage;
   }

   private void createUI(Messager messager)
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

   private double totalTimeTaken;

   private String waitForResult(double maxTimeToWait, String prefix)
   {
      long waitTime = 10;

      while (actualResult.get() == null)
      {
         queryUIResults();
         queryPlannerResults();

         if (totalTimeTaken > maxTimeToWait)
         {
            return prefix + " timed out waiting for a result.\n";
         }

         ThreadTools.sleep(waitTime);
         totalTimeTaken += Conversions.millisecondsToSeconds(waitTime);
      }

      return "";
   }

   private String validateResult(String prefix)
   {
      if (!actualResult.get().validForExecution())
      {
         return prefix + " failed to find a valid result. Result : " + actualResult.get() + "\n";
      }
      else
      {
         return "";
      }
   }

   private String waitForPlan(double maxTimeToWait, String prefix)
   {
      while (actualPlan.get() == null)
      {
         queryUIResults();
         queryPlannerResults();
         long waitTime = 10;

         if (totalTimeTaken > maxTimeToWait)
         {
            return prefix + " timed out waiting on plan.\n";
         }

         ThreadTools.sleep(waitTime);
         totalTimeTaken += Conversions.millisecondsToSeconds(waitTime);
      }

      return "";
   }

   private void queryUIResults()
   {
      if (uiReceivedPlan.get() && uiFootstepPlanReference.get() != null && actualPlan.get() == null)
      {
         if (DEBUG)
            LogTools.info("Received a plan from the UI.");
         actualPlan.set(uiFootstepPlanReference.getAndSet(null));
         uiReceivedPlan.set(false);
      }

      if (uiReceivedResult.get() && uiPlanningResultReference.get() != null)
      {
         if (DEBUG)
            LogTools.info("Received a result " + uiPlanningResultReference.get() + " from the UI.");
         actualResult.set(uiPlanningResultReference.getAndSet(null));
         uiReceivedResult.set(false);
      }
   }

   private void queryPlannerResults()
   {
      if (plannerReceivedPlan.get() && plannerPlanReference.get() != null && expectedPlan.get() == null)
      {
         if (DEBUG)
            LogTools.info("Received a plan from the planner.");
         expectedPlan.set(plannerPlanReference.getAndSet(null));
         plannerReceivedPlan.set(false);
      }

      if (plannerReceivedResult.get() && plannerResultReference.get() != null)
      {
         if (DEBUG)
            LogTools.info("Received a result " + plannerResultReference.get() + " from the planner.");
         expectedResult.set(plannerResultReference.getAndSet(null));
         plannerReceivedResult.set(false);
      }
   }

   private String findPlanAndAssertGoodResult(FootstepPlannerUnitTestDataset dataset)
   {
      totalTimeTaken = 0.0;
      double timeoutMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double maxTimeToWait = 2.0 * timeoutMultiplier * dataset.getTimeout(getPlannerType());
      String datasetName = dataset.getDatasetName();

      String errorMessage = waitForResult(maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      errorMessage = validateResult(datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      errorMessage = waitForPlan(maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      FootstepPlanningResult result = actualResult.getAndSet(null);
      FootstepPlan plan = actualPlan.getAndSet(null);
      errorMessage = assertPlanIsValid(datasetName, result, plan, dataset.getGoal());

      ThreadTools.sleep(1000);
      return errorMessage;
   }

   protected interface DatasetTestRunner
   {
      String testDataset(FootstepPlannerUnitTestDataset dataset);
   }
}

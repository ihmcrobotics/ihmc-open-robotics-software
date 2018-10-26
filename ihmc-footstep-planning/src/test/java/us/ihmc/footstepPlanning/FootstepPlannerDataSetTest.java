package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.*;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.apache.commons.lang3.tuple.Pair;
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
import us.ihmc.javaFXToolkit.messager.Messager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryMessager;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerTypeTopic;

public abstract class FootstepPlannerDataSetTest
{
   protected static final double bambooTimeScaling = 4.0;

   // Whether to start the UI or not.
   protected static boolean VISUALIZE = false;
   // For enabling helpful prints.
   protected static boolean DEBUG = false;
   protected static boolean VERBOSE = false;

   protected FootstepPlannerUI ui = null;
   protected Messager messager = null;

   private final AtomicReference<FootstepPlan> plannerPlanReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> plannerResultReference = new AtomicReference<>(null);
   protected final AtomicReference<Boolean> plannerReceivedPlan = new AtomicReference<>(false);
   protected final AtomicReference<Boolean> plannerReceivedResult = new AtomicReference<>(false);

   protected AtomicReference<FootstepPlan> uiFootstepPlanReference;
   protected AtomicReference<FootstepPlanningResult> uiPlanningResultReference;
   protected final AtomicReference<Boolean> uiReceivedPlan = new AtomicReference<>(false);
   protected final AtomicReference<Boolean> uiReceivedResult = new AtomicReference<>(false);

   protected final AtomicReference<FootstepPlan> expectedPlan = new AtomicReference<>(null);
   protected final AtomicReference<FootstepPlan> actualPlan = new AtomicReference<>(null);
   protected final AtomicReference<FootstepPlanningResult> expectedResult = new AtomicReference<>(null);
   protected final AtomicReference<FootstepPlanningResult> actualResult = new AtomicReference<>(null);

   public abstract FootstepPlannerType getPlannerType();

   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (VISUALIZE)
         messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
      else
         messager = new SharedMemoryMessager(FootstepPlannerMessagerAPI.API);

      setupInternal();

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

      for (int i = 0; i < 100; i++)
         ThreadTools.sleep(10);
   }

   public void setupInternal()
   {

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
      runAssertionsOnAllDatasetsWithoutOcclusions(dataset -> runAssertions(dataset));
   }

   @Test(timeout = 500000)
   @ContinuousIntegrationTest(estimatedDuration = 13.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   public void testDatasetsWithoutOcclusionInDevelopment()
   {
      runAssertionsOnAllDatasetsWithoutOcclusionsInDevelopment(dataset -> runAssertions(dataset));
   }

   protected void runAssertionsOnAllDatasetsWithoutOcclusions(DatasetTestRunner datasetTestRunner)
   {
      List<FootstepPlannerUnitTestDataset> allDatasets = FootstepPlannerIOTools
            .loadAllFootstepPlannerDatasetsWithoutOcclusions(FootstepPlannerDataExporter.class);

      resetAllAtomics();

      runAssertionsOnAllDatasets(datasetTestRunner, allDatasets);
   }

   protected void runAssertionsOnAllDatasetsWithoutOcclusionsInDevelopment(DatasetTestRunner datasetTestRunner)
   {
      List<FootstepPlannerUnitTestDataset> allDatasets = FootstepPlannerIOTools
            .loadAllFootstepPlannerDatasetsWithoutOcclusionsInDevelopment(FootstepPlannerDataExporter.class);

      resetAllAtomics();

      runAssertionsOnAllDatasets(datasetTestRunner, allDatasets);
   }

   public void runAssertionsOnDataset(DatasetTestRunner datasetTestRunner, String datasetName)
   {
      FootstepPlannerUnitTestDataset dataset = FootstepPlannerIOTools.loadDataset(FootstepPlannerDataExporter.class, datasetName);

      Random random = new Random(324);
      dataset.getPlanarRegionsList().getPlanarRegionsAsList().forEach(region -> region.setRegionId(random.nextInt()));

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

      List<Pair<String, String>> failingTestNameAndErrorList = new ArrayList<>();
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
         {
            failingTestNameAndErrorList.add(Pair.of(dataset.getDatasetName(), errorMessagesForCurrentFile));
         }

         if (DEBUG || VERBOSE)
         {
            String result = errorMessagesForCurrentFile.isEmpty() ? "passed" : "failed";
            PrintTools.info(dataset.getDatasetName() + " " + result);
         }

         ThreadTools.sleep(500); // Apparently need to give some time for the prints to appear in the right order.
      }

      Assert.assertTrue("Number of failing datasets: " + failingTestNameAndErrorList.size() + " out of " + allDatasets.size(),
                        failingTestNameAndErrorList.isEmpty());
   }

   public String runAssertions(FootstepPlannerUnitTestDataset dataset)
   {
      submitDataSet(dataset);

      return findPlanAndAssertGoodResult(dataset);
   }

   protected void packPlanningRequest(FootstepPlannerUnitTestDataset dataset, FootstepPlanningRequestPacket packet)
   {
      byte plannerType = getPlannerType().toByte();
      PlanarRegionsListMessage planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(dataset.getPlanarRegionsList());

      packet.getStanceFootPositionInWorld().set(dataset.getStart());
      packet.getGoalPositionInWorld().set(dataset.getGoal());
      packet.setRequestedFootstepPlannerType(plannerType);
      packet.getPlanarRegionsListMessage().set(planarRegions);

      double timeoutMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      packet.setTimeout(timeoutMultiplier * dataset.getTimeout(getPlannerType()));

      packet.setHorizonLength(Double.MAX_VALUE);

      if (dataset.hasGoalOrientation())
         packet.getGoalOrientationInWorld().set(dataset.getGoalOrientation());
      if (dataset.hasStartOrientation())
         packet.getStanceFootOrientationInWorld().set(dataset.getStartOrientation());

      if (DEBUG)
         PrintTools.info("Sending out planning request packet.");
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

   protected void processFootstepPlanningOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      if (DEBUG)
         PrintTools.info("Processed an output from a remote planner.");

      plannerResultReference.set(FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult()));
      plannerPlanReference.set(convertToFootstepPlan(packet.getFootstepDataList()));
      plannerReceivedPlan.set(true);
      plannerReceivedResult.set(true);
   }

   private static FootstepPlan convertToFootstepPlan(FootstepDataListMessage footstepDataListMessage)
   {
      FootstepPlan footstepPlan = new FootstepPlan();

      for (FootstepDataMessage footstepMessage : footstepDataListMessage.getFootstepDataList())
      {
         FramePose3D stepPose = new FramePose3D();
         stepPose.setPosition(footstepMessage.getLocation());
         stepPose.setOrientation(footstepMessage.getOrientation());
         SimpleFootstep footstep = footstepPlan.addFootstep(RobotSide.fromByte(footstepMessage.getRobotSide()), stepPose);

         ConvexPolygon2D foothold = new ConvexPolygon2D();
         for (int i = 0; i < footstepMessage.getPredictedContactPoints2d().size(); i++)
            foothold.addVertex(footstepMessage.getPredictedContactPoints2d().get(i));
         foothold.update();
         footstep.setFoothold(foothold);
      }

      return footstepPlan;
   }

   protected String assertPlansAreValid(String datasetName, FootstepPlanningResult expectedResult, FootstepPlanningResult actualResult,
                                        FootstepPlan expectedPlan, FootstepPlan actualPlan, Point3D goal)
   {
      String errorMessage = "";

      errorMessage += assertTrue(datasetName, "Planning results for " + datasetName + " are not equal: " + expectedResult + " and " + actualResult + ".\n",
                                 expectedResult.equals(actualResult));

      errorMessage += assertPlanIsValid(datasetName, expectedResult, expectedPlan, goal);
      errorMessage += assertPlanIsValid(datasetName, actualResult, actualPlan, goal);

      if (actualResult.validForExecution())
      {
         errorMessage += areFootstepPlansEqual(actualPlan, expectedPlan);
      }

      return errorMessage;
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

   private String areFootstepPlansEqual(FootstepPlan actualPlan, FootstepPlan expectedPlan)
   {
      String errorMessage = "";

      if (actualPlan.getNumberOfSteps() != expectedPlan.getNumberOfSteps())
      {
         errorMessage += "Actual Plan has " + actualPlan.getNumberOfSteps() + ", while Expected Plan has " + expectedPlan.getNumberOfSteps() + ".\n";
      }

      for (int i = 0; i < Math.min(actualPlan.getNumberOfSteps(), expectedPlan.getNumberOfSteps()); i++)
      {
         errorMessage += areFootstepsEqual(i, actualPlan.getFootstep(i), expectedPlan.getFootstep(i));
      }

      return errorMessage;
   }

   private String areFootstepsEqual(int footstepNumber, SimpleFootstep actual, SimpleFootstep expected)
   {
      String errorMessage = "";

      if (!actual.getRobotSide().equals(expected.getRobotSide()))
      {
         errorMessage += "Footsteps " + footstepNumber + " are different robot sides: " + actual.getRobotSide() + " and " + expected.getRobotSide() + ".\n";
      }

      FramePose3D poseA = new FramePose3D();
      FramePose3D poseB = new FramePose3D();

      actual.getSoleFramePose(poseA);
      expected.getSoleFramePose(poseB);

      if (!poseA.epsilonEquals(poseB, 1e-5))
      {
         errorMessage += "Footsteps " + footstepNumber + " have different poses: \n \t" + poseA.toString() + "\n and \n\t " + poseB.toString() + ".\n";
      }

      if (!actual.epsilonEquals(expected, 1e-5))

      {
         errorMessage += "Footsteps " + footstepNumber + " are not equal: \n \t" + actual.toString() + "\n and \n\t " + expected.toString() + ".\n";
      }

      return errorMessage;
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

   public abstract void submitDataSet(FootstepPlannerUnitTestDataset dataset);

   public abstract String findPlanAndAssertGoodResult(FootstepPlannerUnitTestDataset dataset);

   protected static interface DatasetTestRunner
   {
      String testDataset(FootstepPlannerUnitTestDataset dataset);
   }

   protected static interface ConditionChecker
   {
      boolean checkCondition();
   }
}

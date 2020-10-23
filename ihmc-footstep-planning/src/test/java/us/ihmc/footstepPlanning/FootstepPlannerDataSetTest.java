package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.FootstepDataListMessage;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.components.FootstepPathCalculatorModule;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.starter.ApplicationRunner;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.stream.Collectors;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public abstract class FootstepPlannerDataSetTest
{
   private static final double timeout = 240.0;

   // Whether to start the UI or not.
   protected static boolean VISUALIZE = false;
   // For enabling helpful prints.
   protected static boolean DEBUG = true;
   protected static boolean VERBOSE = true;

   private FootstepPlannerUI ui = null;
   private Messager messager = null;

   private final AtomicReference<FootstepPlan> plannerPlanReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> plannerResultReference = new AtomicReference<>(null);
   private final AtomicReference<Boolean> plannerReceivedPlan = new AtomicReference<>(false);
   private final AtomicReference<Boolean> plannerReceivedResult = new AtomicReference<>(false);

   private AtomicReference<FootstepDataListMessage> uiFootstepPlanReference;
   private AtomicReference<FootstepPlanningResult> uiPlanningResultReference;
   private final AtomicReference<Boolean> uiReceivedPlan = new AtomicReference<>(false);
   private final AtomicReference<Boolean> uiReceivedResult = new AtomicReference<>(false);

   private final AtomicReference<FootstepPlan> expectedPlan = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlan> actualPlan = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> expectedResult = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> actualResult = new AtomicReference<>(null);

   private FootstepPathCalculatorModule module = null;

   protected abstract boolean getPlanBodyPath();

   protected abstract boolean getPerformAStarSearch();

   protected abstract String getTestNamePrefix();

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

      messager.registerTopicListener(FootstepPlanResponse, request -> uiReceivedPlan.set(true));
      messager.registerTopicListener(FootstepPlanningResultTopic, request -> uiReceivedResult.set(true));

      uiFootstepPlanReference = messager.createInput(FootstepPlanResponse);
      uiPlanningResultReference = messager.createInput(FootstepPlanningResultTopic);
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      module.stop();
      messager.closeMessager();

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
   public void testDataSets()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
                                                           {
                                                              if(!dataSet.hasPlannerInput())
                                                                 return false;
                                                              if(!dataSet.getPlannerInput().getStepPlannerIsTestable())
                                                                 return false;
                                                              return dataSet.getPlannerInput().containsIterationLimitFlag(getTestNamePrefix().toLowerCase());
                                                           });

      if (VISUALIZE)
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.TestDataSets, dataSets);
         messager.registerTopicListener(FootstepPlannerMessagerAPI.TestDataSetSelected, this::runAssertions);

         AtomicBoolean uiHasClosed = new AtomicBoolean();
         ui.addShutdownHook(() ->
                            {
                               try
                               {
                                  uiHasClosed.set(true);
                                  tearDown();
                               }
                               catch (Exception e)
                               {
                                  e.printStackTrace();
                                  Platform.exit();
                               }
                            });

         while (!uiHasClosed.get())
         {
            ThreadTools.sleep(1000);
         }
      }
      else
      {
         runAssertionsOnAllDatasets(this::runAssertions, dataSets);
      }
   }

   @Test
   @Disabled
   public void testDatasetsInDevelopment()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
                                                           {
                                                              if(!dataSet.hasPlannerInput())
                                                                 return false;
                                                              if(!dataSet.getPlannerInput().getStepPlannerIsInDevelopment())
                                                                 return false;
                                                              return dataSet.getPlannerInput().containsIterationLimitFlag(getTestNamePrefix().toLowerCase());
                                                           });
      runAssertionsOnAllDatasets(this::runAssertions, dataSets);
   }

   public void runAssertionsOnDataset(Function<DataSet, String> dataSetTester, DataSetName dataSetName)
   {
      DataSet dataset = DataSetIOTools.loadDataSet(dataSetName);
      resetAllAtomics();
      String errorMessages = dataSetTester.apply(dataset);
      Assert.assertTrue("Errors:" + errorMessages, errorMessages.isEmpty());
   }

   protected void runAssertionsOnAllDatasets(Function<DataSet, String> dataSetTester, List<DataSet> allDatasets)
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
         DataSet dataset = allDatasets.get(i);
         if (DEBUG || VERBOSE)
            LogTools.info("Testing file: " + dataset.getName());

         numbberOfTestedSets++;
         resetAllAtomics();
         String errorMessagesForCurrentFile = dataSetTester.apply(dataset);
         if (!errorMessagesForCurrentFile.isEmpty())
         {
            numberOfFailingTests++;
            failingDatasets.add(dataset.getName());
         }

         if (DEBUG || VERBOSE)
         {
            String result = errorMessagesForCurrentFile.isEmpty() ? "passed" : "failed";
            LogTools.info(dataset.getName() + " " + result);
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

   protected String runAssertions(DataSet dataset)
   {
      ThreadTools.sleep(1000);
      packPlanningRequest(dataset, messager);
      return findPlanAndAssertGoodResult(dataset);
   }

   protected void packPlanningRequest(DataSet dataset, Messager messager)
   {
      PlannerInput plannerInput = dataset.getPlannerInput();

      double startYaw = plannerInput.hasStartOrientation() ? plannerInput.getStartYaw() : 0.0;
      double goalYaw = plannerInput.hasGoalOrientation() ? plannerInput.getGoalYaw() : 0.0;
      SideDependentList<Pose3D> startSteps = PlannerTools.createSquaredUpFootsteps(plannerInput.getStartPosition(),
                                                                                   startYaw,
                                                                                   module.getPlanningModule()
                                                                                         .getFootstepPlannerParameters()
                                                                                         .getIdealFootstepWidth());
      SideDependentList<Pose3D> goalSteps = PlannerTools.createSquaredUpFootsteps(plannerInput.getGoalPosition(),
                                                                                  goalYaw,
                                                                                  module.getPlanningModule()
                                                                                        .getFootstepPlannerParameters()
                                                                                        .getIdealFootstepWidth());
      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootPose, startSteps.get(RobotSide.LEFT));
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootPose, startSteps.get(RobotSide.RIGHT));
      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootGoalPose, goalSteps.get(RobotSide.LEFT));
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootGoalPose, goalSteps.get(RobotSide.RIGHT));

      messager.submitMessage(PlanBodyPath, getPlanBodyPath());
      messager.submitMessage(PerformAStarSearch, getPerformAStarSearch());

      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, dataset.getPlanarRegionsList());

      int maxIterations = 300; // plannerInput.getIterationLimitFlag(getTestNamePrefix().toLowerCase());
      messager.submitMessage(FootstepPlannerMessagerAPI.MaxIterations, maxIterations);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeout, timeout);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerHorizonLength, Double.MAX_VALUE);
      messager.submitMessage(FootstepPlannerMessagerAPI.ComputePath, true);

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
            SideDependentList<ConvexPolygon2D> defaultFootPolygons = PlannerTools.createDefaultFootPolygons();
            SideDependentList<List<Point2D>> defaultContactPoints = new SideDependentList<>();
            defaultContactPoints.set(side -> defaultFootPolygons.get(side).getVertexBufferView().stream().map(Point2D::new).collect(Collectors.toList()));

            ui = FootstepPlannerUI.createMessagerUI(stage, (SharedMemoryJavaFXMessager) messager, true, defaultContactPoints);
            ui.show();
         }

         @Override
         public void stop()
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

         String resultAgain = validateResult(prefix);
         if (!resultAgain.isEmpty())
            return resultAgain;
      }

      return "";
   }

   private void queryUIResults()
   {
      if (uiReceivedPlan.get() && uiFootstepPlanReference.get() != null && actualPlan.get() == null)
      {
         if (DEBUG)
            LogTools.info("Received a plan from the UI.");
         actualPlan.set(FootstepDataMessageConverter.convertToFootstepPlan(uiFootstepPlanReference.getAndSet(null)));
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

   private String findPlanAndAssertGoodResult(DataSet dataset)
   {
      PlannerInput plannerInput = dataset.getPlannerInput();
      totalTimeTaken = 0.0;
      String datasetName = dataset.getName();

      String errorMessage = waitForResult(timeout, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      errorMessage = validateResult(datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      errorMessage = waitForPlan(timeout, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      FootstepPlanningResult result = actualResult.getAndSet(null);
      FootstepPlan plan = actualPlan.getAndSet(null);
      errorMessage = assertPlanIsValid(datasetName, result, plan, plannerInput.getGoalPosition());

      ThreadTools.sleep(1000);
      return errorMessage;
   }
}

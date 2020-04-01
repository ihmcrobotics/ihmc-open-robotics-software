package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.starter.ApplicationRunner;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersBasics;
import us.ihmc.quadrupedFootstepPlanning.ui.PawStepPlannerUI;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public abstract class PawStepPlannerDataSetTest
{
   private static final double defaultBestEffortTimeout = 1.0;
   protected static final double bambooTimeScaling = 4.0;
   private static final double defaultHorizonLength = 100.0;

   private static final QuadrantDependentList<AppearanceDefinition> colorDefinitions = new QuadrantDependentList<>(YoAppearance.Red(), YoAppearance.Green(),
                                                                                                                   YoAppearance.DarkRed(),
                                                                                                                   YoAppearance.DarkGreen());

   // Whether to start the UI or not.
   protected static boolean VISUALIZE = false;
   // For enabling helpful prints.
   protected static boolean DEBUG = false;
   protected static boolean VERBOSE = false;

   private PawStepPlannerUI ui = null;
   protected Messager messager = null;

   protected QuadrupedXGaitSettings xGaitSettings = null;
   protected PawStepPlannerParametersBasics plannerParameters = null;
   private BodyPathAndPawPlanner planner = null;

   protected abstract QuadrupedXGaitSettings getXGaitSettings();
   protected abstract PawStepPlannerParametersBasics getPlannerParameters();

   protected abstract BodyPathAndPawPlanner createPlanner();

   @BeforeEach
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (VISUALIZE)
         messager = new SharedMemoryJavaFXMessager(PawStepPlannerMessagerAPI.API);
      else
         messager = new SharedMemoryMessager(PawStepPlannerMessagerAPI.API);

      planner = createPlanner();
      if (xGaitSettings == null)
         xGaitSettings = getXGaitSettings();
      if (plannerParameters == null)
         plannerParameters = getPlannerParameters();

      try
      {
         messager.startMessager();
      }
      catch (Exception e)
      {
         throw new RuntimeException("Failed to start messager.");
      }

      messager.submitMessage(PawStepPlannerMessagerAPI.XGaitSettingsTopic, xGaitSettings);

      if (VISUALIZE)
      {
         createUI(messager);
      }

      ThreadTools.sleep(1000);
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      messager.closeMessager();
      if (ui != null)
         ui.stop();

      ui = null;
      messager = null;
      planner = null;
   }

   private void createUI(Messager messager)
   {
      ApplicationRunner.runApplication(new Application()
      {
         @Override
         public void start(Stage stage) throws Exception
         {
            ui = PawStepPlannerUI.createMessagerUI(stage, (SharedMemoryJavaFXMessager) messager);
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

   @Test
   public void testDataSets()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
                                                           {
                                                              if (!dataSet.hasPlannerInput())
                                                                 return false;
                                                              return dataSet.getPlannerInput().getQuadrupedPlannerIsTestable();
                                                           });
      runAssertionsOnAllDatasets(dataSets);
   }

   @Disabled
   @Test
   public void runInDevelopmentTests()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
                                                           {
                                                              if (!dataSet.hasPlannerInput())
                                                                 return false;
                                                              return dataSet.getPlannerInput().getQuadrupedPlannerIsInDevelopment();
                                                           });
      runAssertionsOnAllDatasets(dataSets);
   }

   @Test
   public void testSimpleForwardPoint()
   {

      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();
      goalPose.setPosition(1.5, 0.5, 0.0);
      goalPose.setOrientationYawPitchRoll(-Math.PI / 4.0, 0.0, 0.0);


      PawStepPlannerStart start = new PawStepPlannerStart();
      PawStepPlannerGoal goal = new PawStepPlannerGoal();
      start.setStartPose(startPose);
      goal.setGoalPose(goalPose);

      planner.setPlanarRegionsList(null);
      planner.setStart(start);
      planner.setGoal(goal);
      planner.setTimeout(20.0);
      planner.setBestEffortTimeout(defaultBestEffortTimeout);

      xGaitSettings.setEndPhaseShift(180.0);
      xGaitSettings.setQuadrupedSpeed(QuadrupedSpeed.MEDIUM);
      xGaitSettings.getTrotMediumTimings().setMaxSpeed(0.6);
      xGaitSettings.getTrotMediumTimings().setEndDoubleSupportDuration(0.15);
      xGaitSettings.getTrotMediumTimings().setStepDuration(0.3);
      xGaitSettings.setStanceLength(1.1);
      xGaitSettings.setStanceWidth(0.2);

      messager.submitMessage(PawStepPlannerMessagerAPI.XGaitSettingsTopic, xGaitSettings);
      messager.submitMessage(PawStepPlannerMessagerAPI.PlanarRegionDataTopic, null);
      messager.submitMessage(PawStepPlannerMessagerAPI.StartPositionTopic, new Point3D(startPose.getPosition()));
      messager.submitMessage(PawStepPlannerMessagerAPI.GoalPositionTopic, new Point3D(goalPose.getPosition()));
      messager.submitMessage(PawStepPlannerMessagerAPI.StartOrientationTopic, new Quaternion(startPose.getOrientation()));
      messager.submitMessage(PawStepPlannerMessagerAPI.GoalOrientationTopic, new Quaternion(goalPose.getOrientation()));
      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerHorizonLengthTopic, defaultHorizonLength);
      //      planner.setHorizonLengthTopic(Double.MAX_VALUE);

      PawStepPlanningResult pathResult = planner.planPath();
      assertTrue("Path plan is invalid. Got path result " + pathResult, pathResult.validForExecution());

      PawStepPlanningResult planResult = planner.plan();
      assertTrue("Footstep plan is invalid. Got path result " + planResult, planResult.validForExecution());

      messager.submitMessage(PawStepPlannerMessagerAPI.FootstepPlanTopic, planner.getPlan());

      String errorMessage = assertPlanIsValid("", planner.getPlan(), goalPose.getPosition(), goalPose.getYaw());
      assertTrue(errorMessage, errorMessage.isEmpty());

      if (VISUALIZE)
         ThreadTools.sleepForever();
   }



   private void runAssertionsOnAllDatasets(List<DataSet> allDatasets)
   {
      if (VERBOSE || DEBUG)
         LogTools.info("Unit test files found: " + allDatasets.size());

      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      int numberOfFailingTests = 0;
      List<String> failingDatasets = new ArrayList<>();
      List<String> failingMessages = new ArrayList<>();
      int numbberOfTestedSets = 0;
      for (int i = 0; i < allDatasets.size(); i++)
      {
         DataSet dataset = allDatasets.get(i);
         if (DEBUG || VERBOSE)
            LogTools.info("Testing file: " + dataset.getName());

         numbberOfTestedSets++;
         String errorMessagesForCurrentFile = runAssertions(dataset);
         if (!errorMessagesForCurrentFile.isEmpty())
         {
            numberOfFailingTests++;
            failingDatasets.add(dataset.getName());
            failingMessages.add(errorMessagesForCurrentFile);
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
         message += "\n" + failingDatasets.get(i) + " : " + failingMessages.get(i);
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

   protected String runAssertions(DataSetName dataSetName)
   {
      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);
      return runAssertions(dataSet);
   }

   protected String runAssertions(DataSet dataset)
   {
      ThreadTools.sleep(1000);
      packPlanningRequest(dataset);
      String errorMessage = findPlanAndAssertGoodResult(dataset);

//      visualizePlan(planner.getPlan(), dataset.getPlanarRegionsList(), dataset.getPlannerInput().getQuadrupedStartPosition(),
//                    dataset.getPlannerInput().getQuadrupedGoalPosition());
//
      return errorMessage;
   }

   protected void packPlanningRequest(DataSet dataset)
   {
      PlannerInput plannerInput = dataset.getPlannerInput();
      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();

      startPose.setPosition(plannerInput.getQuadrupedStartPosition());
      goalPose.setPosition(plannerInput.getQuadrupedGoalPosition());

      if(plannerInput.getHasQuadrupedStartYaw())
         startPose.setOrientation(new Quaternion(plannerInput.getQuadrupedStartYaw(), 0.0, 0.0));
      if(plannerInput.getHasQuadrupedGoalYaw())
         goalPose.setOrientation(new Quaternion(plannerInput.getQuadrupedGoalYaw(), 0.0, 0.0));

      double timeMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double timeout = timeMultiplier * plannerInput.getQuadrupedTimeout();

      PawStepPlannerStart start = new PawStepPlannerStart();
      PawStepPlannerGoal goal = new PawStepPlannerGoal();
      start.setStartPose(startPose);
      goal.setGoalPose(goalPose);

      planner.setPlanarRegionsList(dataset.getPlanarRegionsList());
      planner.setStart(start);
      planner.setGoal(goal);
      planner.setTimeout(timeout);
      planner.setBestEffortTimeout(defaultBestEffortTimeout);
      planner.setPlanningHorizonLength(defaultHorizonLength);

      messager.submitMessage(PawStepPlannerMessagerAPI.XGaitSettingsTopic, xGaitSettings);
      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerParametersTopic, plannerParameters);
      messager.submitMessage(PawStepPlannerMessagerAPI.PlanarRegionDataTopic, dataset.getPlanarRegionsList());
      messager.submitMessage(PawStepPlannerMessagerAPI.StartPositionTopic, new Point3D(startPose.getPosition()));
      messager.submitMessage(PawStepPlannerMessagerAPI.GoalPositionTopic, new Point3D(goalPose.getPosition()));
      messager.submitMessage(PawStepPlannerMessagerAPI.StartOrientationTopic, new Quaternion(startPose.getOrientation()));
      messager.submitMessage(PawStepPlannerMessagerAPI.GoalOrientationTopic, new Quaternion(goalPose.getOrientation()));
      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerHorizonLengthTopic, defaultHorizonLength);
      //      planner.setHorizonLengthTopic(Double.MAX_VALUE);

      if (DEBUG)
         LogTools.info("Set planner parameters.");
   }

   private String findPlanAndAssertGoodResult(DataSet dataset)
   {
      String datasetName = dataset.getName();

      PawStepPlanningResult pathResult = planner.planPath();
      if (!pathResult.validForExecution())
         return "Path plan for " + datasetName + " is invalid. Got path result " + pathResult;

      PawStepPlanningResult planResult = planner.plan();
      if (!planResult.validForExecution())
         return "Footstep plan for " + datasetName + " is invalid. Got plan result " + planResult;

      messager.submitMessage(PawStepPlannerMessagerAPI.FootstepPlanTopic, planner.getPlan());

      PlannerInput plannerInput = dataset.getPlannerInput();
      String errorMessage = assertPlanIsValid(datasetName, planner.getPlan(), plannerInput.getQuadrupedGoalPosition(), plannerInput.getQuadrupedGoalYaw());

      ThreadTools.sleep(1000);
      return errorMessage;
   }

   private static String assertPlanIsValid(String datasetName, PawStepPlan plannedSteps, Point3DReadOnly goalPosition, double goalYaw)
   {
      QuadrantDependentList<Point3DBasics> finalSteps = getFinalStepPositions(plannedSteps);

      Point3D centerPoint = new Point3D();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         centerPoint.add(finalSteps.get(robotQuadrant));
      }

      double nominalYaw = PawNode.computeNominalYaw(finalSteps.get(RobotQuadrant.FRONT_LEFT).getX(), finalSteps.get(RobotQuadrant.FRONT_LEFT).getY(),
                                                    finalSteps.get(RobotQuadrant.FRONT_RIGHT).getX(), finalSteps.get(RobotQuadrant.FRONT_RIGHT).getY(),
                                                    finalSteps.get(RobotQuadrant.HIND_LEFT).getX(), finalSteps.get(RobotQuadrant.HIND_LEFT).getY(),
                                                    finalSteps.get(RobotQuadrant.HIND_RIGHT).getX(), finalSteps.get(RobotQuadrant.HIND_RIGHT).getY());

      centerPoint.scale(0.25);

      String errorMessage = "";
      if (goalPosition.distanceXY(centerPoint) > 3.0 * PawNode.gridSizeXY)
         errorMessage = datasetName + " did not reach goal position. Made it to " + centerPoint + ", trying to get to " + goalPosition;
      if (!Double.isNaN(goalYaw))
      {
         double yawError = AngleTools.computeAngleDifferenceMinusPiToPi(goalYaw, nominalYaw);
         if (yawError > PawNode.gridSizeYaw)
            errorMessage = datasetName + " did not reach goal yaw. Made it to " + nominalYaw + ", trying to get to " + goalYaw;
      }

      errorMessage += checkStepOrder(datasetName, plannedSteps);

      if ((VISUALIZE || DEBUG) && !errorMessage.isEmpty())
         LogTools.error(errorMessage);

      return errorMessage;
   }

   private static QuadrantDependentList<Point3DBasics> getFinalStepPositions(PawStepPlan plannedSteps)
   {
      QuadrantDependentList<Point3DBasics> finalSteps = new QuadrantDependentList<>();
      for (int i = plannedSteps.getNumberOfSteps() - 1; i >= 0; i--)
      {
         QuadrupedTimedStep step = plannedSteps.getPawStep(i);
         if (finalSteps.containsKey(step.getRobotQuadrant()))
            continue;
         else
            finalSteps.put(step.getRobotQuadrant(), new Point3D(step.getGoalPosition()));
      }

      return finalSteps;
   }

   private static String checkStepOrder(String dataseName, PawStepPlan plannedSteps)
   {
      String errorMessage = "";
      RobotQuadrant previousMovingQuadrant = plannedSteps.getPawStep(0).getRobotQuadrant();
      for (int i = 1; i < plannedSteps.getNumberOfSteps(); i++)
      {
         RobotQuadrant movingQuadrant = plannedSteps.getPawStep(i).getRobotQuadrant();
         if (previousMovingQuadrant.getNextRegularGaitSwingQuadrant() != movingQuadrant)
            errorMessage += dataseName + " step " + i + " in the plan is out of order.\n";

         previousMovingQuadrant = movingQuadrant;
      }

      return errorMessage;
   }

   private void visualizePlan(PawStepPlan plan, PlanarRegionsList planarRegionsList, Point3DReadOnly start, Point3DReadOnly goal)
   {
      if (!VISUALIZE || ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         return;

      SimulationConstructionSet scs = new SimulationConstructionSet();

      Graphics3DObject graphics3DObject = new Graphics3DObject();
      if (planarRegionsList != null)
         Graphics3DObjectTools.addPlanarRegionsList(graphics3DObject, planarRegionsList, YoAppearance.White(), YoAppearance.Grey(), YoAppearance.DarkGray());
      scs.setGroundVisible(false);

      graphics3DObject.identity();
      graphics3DObject.translate(start);
      graphics3DObject.translate(0.0, 0.0, 0.05);
      graphics3DObject.addCone(0.3, 0.05, YoAppearance.Blue());

      graphics3DObject.identity();
      graphics3DObject.translate(goal);
      graphics3DObject.translate(0.0, 0.0, 0.05);
      graphics3DObject.addCone(0.3, 0.05, YoAppearance.Black());

      if (plan != null)
      {
         for (int i = 0; i < plan.getNumberOfSteps(); i++)
         {
            Point3DReadOnly point = plan.getPawStep(i).getGoalPosition();
            AppearanceDefinition appearanceDefinition = colorDefinitions.get(plan.getPawStep(i).getRobotQuadrant());

            graphics3DObject.identity();
            graphics3DObject.translate(point);
            graphics3DObject.addSphere(0.1, appearanceDefinition);

         }
      }

      scs.addStaticLinkGraphics(graphics3DObject);

      scs.setCameraFix(0.0, 0.0, 0.0);
      scs.setCameraPosition(-0.001, 0.0, 15.0);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }
}

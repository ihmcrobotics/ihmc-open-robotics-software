package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.junit.Test;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
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
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.tools.FootstepPlannerIOTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.quadrupedFootstepPlanning.ui.ApplicationRunner;
import us.ihmc.quadrupedFootstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.util.ArrayList;
import java.util.List;

public abstract class FootstepPlannerDataSetTest
{
   protected static final double bambooTimeScaling = 4.0;
   private static final double epsilon = 1e-3;

   private static final QuadrantDependentList<AppearanceDefinition> colorDefinitions = new QuadrantDependentList<>(YoAppearance.Red(), YoAppearance.Green(),
                                                                                                                   YoAppearance.DarkRed(),
                                                                                                                   YoAppearance.DarkGreen());

   // Whether to start the UI or not.
   protected static boolean VISUALIZE = false;
   // For enabling helpful prints.
   protected static boolean DEBUG = true;
   protected static boolean VERBOSE = true;

   private FootstepPlannerUI ui = null;
   protected Messager messager = null;

   private QuadrupedBodyPathAndFootstepPlanner planner = null;

   protected abstract FootstepPlannerType getPlannerType();

   protected abstract QuadrupedBodyPathAndFootstepPlanner createPlanner();

   @BeforeEach
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (VISUALIZE)
         messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
      else
         messager = new SharedMemoryMessager(FootstepPlannerMessagerAPI.API);

      planner = createPlanner();

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

   @Test
   public void testDatasetsWithoutOcclusion()
   {
      List<FootstepPlannerUnitTestDataset> allDatasets = FootstepPlannerIOTools.loadAllFootstepPlannerDatasetsWithoutOcclusions(FootstepPlannerIOTools.class);
      runAssertionsOnAllDatasets(this::runAssertions, allDatasets);
   }

   @Disabled
   @Test
   public void testDatasetsWithoutOcclusionInDevelopment()
   {
      List<FootstepPlannerUnitTestDataset> allDatasets = FootstepPlannerIOTools
            .loadAllFootstepPlannerDatasetsWithoutOcclusionsInDevelopment(FootstepPlannerIOTools.class);
      runAssertionsOnAllDatasets(this::runAssertions, allDatasets);
   }

   protected void runAssertionsOnDataset(DatasetTestRunner datasetTestRunner, String datasetName)
   {
      FootstepPlannerUnitTestDataset dataset = FootstepPlannerIOTools.loadDataset(FootstepPlannerIOTools.class, datasetName);

      String errorMessages = datasetTestRunner.testDataset(dataset);
//      Assert.assertTrue("Errors:" + errorMessages, errorMessages.isEmpty());
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

         if (!dataset.getTypes().contains(getPlannerType()))
         {
            if (DEBUG || VERBOSE)
               LogTools.info(dataset.getDatasetName() + " does not contain planner type " + getPlannerType() + ", skipping");
            continue;
         }

         numbberOfTestedSets++;
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
      packPlanningRequest(dataset);
      String errorMessage = findPlanAndAssertGoodResult(dataset);

//      visualizePlan(planner.getPlan(), dataset.getPlanarRegionsList(), dataset.getStart(), dataset.getGoal());

      return errorMessage;
   }

   protected void packPlanningRequest(FootstepPlannerUnitTestDataset dataset)
   {
      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();
      startPose.setPosition(dataset.getStart());
      goalPose.setPosition(dataset.getGoal());
      if (dataset.hasStartOrientation())
         startPose.setOrientation(dataset.getStartOrientation());
      if (dataset.hasGoalOrientation())
         goalPose.setOrientation(dataset.getGoalOrientation());

      double timeMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double timeout = timeMultiplier * dataset.getTimeout(getPlannerType());

      QuadrupedFootstepPlannerStart start = new QuadrupedFootstepPlannerStart();
      QuadrupedFootstepPlannerGoal goal = new QuadrupedFootstepPlannerGoal();
      start.setStartPose(startPose);
      goal.setGoalPose(goalPose);

      planner.setStart(start);
      planner.setGoal(goal);
      planner.setPlanarRegionsList(dataset.getPlanarRegionsList());
      planner.setTimeout(timeout);

      messager.submitMessage(FootstepPlannerMessagerAPI.StartPositionTopic, new Point3D(startPose.getPosition()));
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalPositionTopic, new Point3D(goalPose.getPosition()));
      messager.submitMessage(FootstepPlannerMessagerAPI.StartOrientationTopic, new Quaternion(goalPose.getOrientation()));
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalOrientationTopic, new Quaternion(goalPose.getOrientation()));
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionDataTopic, dataset.getPlanarRegionsList());
      //      planner.setHorizonLengthTopic(Double.MAX_VALUE);

      if (DEBUG)
         LogTools.info("Set planner parameters.");
   }

   private String findPlanAndAssertGoodResult(FootstepPlannerUnitTestDataset dataset)
   {
      String datasetName = dataset.getDatasetName();

      FootstepPlanningResult pathResult = planner.planPath();
      if (!pathResult.validForExecution())
         return "Path plan for " + datasetName + " is invalid.";

      FootstepPlanningResult planResult = planner.plan();
      if (!planResult.validForExecution())
         return "Footstep plan for " + datasetName + " is invalid.";

      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanTopic, planner.getPlan());

      String errorMessage = assertPlanIsValid(datasetName, planner.getPlan(), dataset.getGoal(), dataset.getGoalOrientation());


      ThreadTools.sleep(1000);
      return errorMessage;
   }

   private static String assertPlanIsValid(String datasetName, FootstepPlan plannedSteps, Point3DReadOnly goalPosition, Quaternion goalOrientation)
   {
      QuadrantDependentList<Point3DBasics> finalSteps = getFinalStepPositions(plannedSteps);

      Point3D centerPoint = new Point3D();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         centerPoint.add(finalSteps.get(robotQuadrant));
      }

      double nominalYaw = FootstepNode.computeNominalYaw(finalSteps.get(RobotQuadrant.FRONT_LEFT).getX(), finalSteps.get(RobotQuadrant.FRONT_LEFT).getY(),
                                                         finalSteps.get(RobotQuadrant.FRONT_RIGHT).getX(), finalSteps.get(RobotQuadrant.FRONT_RIGHT).getY(),
                                                         finalSteps.get(RobotQuadrant.HIND_LEFT).getX(), finalSteps.get(RobotQuadrant.HIND_LEFT).getY(),
                                                         finalSteps.get(RobotQuadrant.HIND_RIGHT).getX(), finalSteps.get(RobotQuadrant.HIND_RIGHT).getY());

      centerPoint.scale(0.25);

      String errorMessage = "";
      if (!goalPosition.epsilonEquals(centerPoint, epsilon))
         errorMessage = datasetName + " did not reach goal position. Made it to " + centerPoint + ", trying to get to " + goalPosition;
      if (goalOrientation != null)
      {
         double goalYaw = goalOrientation.getYaw();
         if (!MathTools.epsilonEquals(goalYaw, nominalYaw, 0.02))
            errorMessage = datasetName + " did not reach goal yaw. Made it to " + nominalYaw + ", trying to get to " + goalYaw;
      }

      if ((VISUALIZE || DEBUG) && !errorMessage.isEmpty())
         LogTools.error(errorMessage);

      return errorMessage;
   }

   private static QuadrantDependentList<Point3DBasics> getFinalStepPositions(FootstepPlan plannedSteps)
   {
      QuadrantDependentList<Point3DBasics> finalSteps = new QuadrantDependentList<>();
      for (int i = plannedSteps.getNumberOfSteps() - 1; i >= 0; i--)
      {
         QuadrupedTimedStep step = plannedSteps.getFootstep(i);
         if (finalSteps.containsKey(step.getRobotQuadrant()))
            continue;
         else
            finalSteps.put(step.getRobotQuadrant(), step.getGoalPosition());
      }

      return finalSteps;
   }

   protected interface DatasetTestRunner
   {
      String testDataset(FootstepPlannerUnitTestDataset dataset);
   }

   private void visualizePlan(FootstepPlan plan, PlanarRegionsList planarRegionsList, Point3DReadOnly start, Point3DReadOnly goal)
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
            Point3DReadOnly point = plan.getFootstep(i).getGoalPosition();
            AppearanceDefinition appearanceDefinition = colorDefinitions.get(plan.getFootstep(i).getRobotQuadrant());

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

package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawPlan;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawPlanningResult;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawPlannerGoal;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawPlannerStart;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.DefaultPawPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.visualization.QuadrupedAStarPawPlannerVisualizer;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.assertEquals;

public class AStarPawPlannerTest
{
   private static final double epsilon = 1e-3;
   private static boolean visualize = false;
   private static boolean activelyVisualize = false;

   private static final QuadrantDependentList<AppearanceDefinition> colorDefinitions = new QuadrantDependentList<>(YoAppearance.Red(), YoAppearance.Green(), YoAppearance.DarkRed(), YoAppearance.DarkGreen());

   @BeforeEach
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      activelyVisualize = activelyVisualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   }

   @Test
   public void testSimpleWalkForward()
   {
      PlanarRegionsList planarRegionsList = null;

      double timeout = 10.0;
      double stanceLength = 1.0;
      double stanceWidth = 0.5;
      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();
      goalPose.setPosition(2.0, 0.0, 0.0);

      runTest(stanceLength, stanceWidth, startPose, goalPose, planarRegionsList, timeout);
   }

   @Test
   public void testSimpleWalkBackward()
   {
      PlanarRegionsList planarRegionsList = null;

      double timeout = 10.0;
      double stanceLength = 1.0;
      double stanceWidth = 0.5;
      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();
      goalPose.setPosition(-2.0, 0.0, 0.0);

      runTest(stanceLength, stanceWidth, startPose, goalPose, planarRegionsList, timeout, RobotQuadrant.HIND_LEFT);
   }

   @Test
   public void testTurnInPlace()
   {
      PlanarRegionsList planarRegionsList = null;

      double timeout = 10.0;
      double stanceLength = 1.0;
      double stanceWidth = 0.5;
      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();
      goalPose.setPosition(0.0, 0.0, Math.PI);
      goalPose.setOrientationYawPitchRoll(Math.PI, 0.0, 0.0);

      runTest(stanceLength, stanceWidth, startPose, goalPose, planarRegionsList, timeout);
   }

   @Test
   public void testWalkAndTurn()
   {
      double timeout = 10.0;
      double stanceLength = 1.0;
      double stanceWidth = 0.5;
      PlanarRegionsList planarRegionsList = null;

      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();
      goalPose.setPosition(2.5, 2.5, 0.0);
      goalPose.setOrientationYawPitchRoll(Math.PI / 3.0, 0.0, 0.0);

      runTest(stanceLength, stanceWidth, startPose, goalPose, planarRegionsList, timeout);
   }

   @Test
   public void testSimpleForwardPoint()
   {
      double timeout = 20.0;
      double stanceLength = 1.0;
      double stanceWidth = 0.5;
      PlanarRegionsList planarRegionsList = null;

      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();
      goalPose.setPosition(1.5, 0.5, 0.0);
      goalPose.setOrientationYawPitchRoll(-Math.PI / 4.0, 0.0, 0.0);

      runTest(stanceLength, stanceWidth, startPose, goalPose, planarRegionsList, timeout);
   }

   @Test
   public void testEnvironment0()
   {
      double timeout = 10.0;
      double stanceLength = 0.8;
      double stanceWidth = 0.4;

      DataSet dataSet = DataSetIOTools.loadDataSet(DataSetName._20190327_163532_QuadrupedEnvironment0);
      PlanarRegionsList planarRegionsList = dataSet.getPlanarRegionsList();

      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();

      PlannerInput plannerInput = dataSet.getPlannerInput();
      startPose.setPosition(plannerInput.getStartPosition());
      startPose.setOrientationYawPitchRoll(plannerInput.getQuadrupedStartYaw(), 0.0, 0.0);

      goalPose.setPosition(plannerInput.getGoalPosition());
      goalPose.setOrientationYawPitchRoll(plannerInput.getQuadrupedGoalYaw(), 0.0, 0.0);

      runTest(stanceLength, stanceWidth, startPose, goalPose, planarRegionsList, timeout);
   }

   @Test
   public void testEnvironment1()
   {
      double timeout = 10.0;
      double stanceLength = 0.8;
      double stanceWidth = 0.4;


      DataSet dataSet = DataSetIOTools.loadDataSet(DataSetName._20190327_174535_QuadrupedEnvironment1);
      PlanarRegionsList planarRegionsList = dataSet.getPlanarRegionsList();

      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();

      PlannerInput plannerInput = dataSet.getPlannerInput();
      startPose.setPosition(plannerInput.getStartPosition());
      startPose.setOrientationYawPitchRoll(plannerInput.getQuadrupedStartYaw(), 0.0, 0.0);

      goalPose.setPosition(plannerInput.getGoalPosition());
      goalPose.setOrientationYawPitchRoll(plannerInput.getQuadrupedGoalYaw(), 0.0, 0.0);

      runTest(stanceLength, stanceWidth, startPose, goalPose, planarRegionsList, timeout);
   }

   @Test
   public void testEnvironment2()
   {
      double timeout = 10.0;
      double stanceLength = 0.8;
      double stanceWidth = 0.4;

      DataSet dataSet = DataSetIOTools.loadDataSet(DataSetName._20190327_175120_QuadrupedEnvironment2);
      PlanarRegionsList planarRegionsList = dataSet.getPlanarRegionsList();

      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();

      PlannerInput plannerInput = dataSet.getPlannerInput();
      startPose.setPosition(plannerInput.getStartPosition());
      startPose.setOrientationYawPitchRoll(plannerInput.getQuadrupedStartYaw(), 0.0, 0.0);

      goalPose.setPosition(plannerInput.getGoalPosition());
      goalPose.setOrientationYawPitchRoll(plannerInput.getQuadrupedGoalYaw(), 0.0, 0.0);

      runTest(stanceLength, stanceWidth, startPose, goalPose, planarRegionsList, timeout);
   }

   @Test
   @Disabled
   public void testEnvironment3()
   {
      double timeout = 10.0;
      double stanceLength = 0.8;
      double stanceWidth = 0.4;

      DataSet dataSet = DataSetIOTools.loadDataSet(DataSetName._20190327_175227_QuadrupedEnvironment3);
      PlanarRegionsList planarRegionsList = dataSet.getPlanarRegionsList();

      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();

      PlannerInput plannerInput = dataSet.getPlannerInput();
      startPose.setPosition(plannerInput.getStartPosition());
      startPose.setOrientationYawPitchRoll(plannerInput.getQuadrupedStartYaw(), 0.0, 0.0);

      goalPose.setPosition(plannerInput.getGoalPosition());
      goalPose.setOrientationYawPitchRoll(plannerInput.getQuadrupedGoalYaw(), 0.0, 0.0);

      runTest(stanceLength, stanceWidth, startPose, goalPose, planarRegionsList, timeout);
   }

   private void runTest(double stanceLength, double stanceWidth, FramePose3D startPose, FramePose3D goalPose, PlanarRegionsList planarRegionsList, double timeout)
   {
      runTest(stanceLength, stanceWidth, startPose, goalPose, planarRegionsList, timeout, null);
   }

   private void runTest(double stanceLength, double stanceWidth, FramePose3D startPose, FramePose3D goalPose, PlanarRegionsList planarRegionsList,
                        double timeout, RobotQuadrant initialQuadrant)
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setStanceLength(stanceLength);
      xGaitSettings.setStanceWidth(stanceWidth);
      xGaitSettings.setEndPhaseShift(90.0);
      xGaitSettings.setQuadrupedSpeed(QuadrupedSpeed.MEDIUM);
      xGaitSettings.getAmbleMediumTimings().setStepDuration(0.4);
      xGaitSettings.getAmbleMediumTimings().setEndDoubleSupportDuration(0.35);
      PawPlannerParameters parameters = new DefaultPawPlannerParameters();
      QuadrupedAStarPawPlannerVisualizer visualizer;
      if (activelyVisualize)
         visualizer = new QuadrupedAStarPawPlannerVisualizer(planarRegionsList);
      else
         visualizer = null;
      AStarPawPlanner planner = AStarPawPlanner.createPlanner(parameters, xGaitSettings, visualizer, registry);

      PawPlannerStart start = new PawPlannerStart();
      PawPlannerGoal goal = new PawPlannerGoal();
      if (initialQuadrant != null)
         start.setInitialQuadrant(initialQuadrant);
      start.setStartPose(startPose);
      goal.setGoalPose(goalPose);

      planner.setStart(start);
      planner.setGoal(goal);
      planner.setTimeout(timeout);
      planner.setPlanarRegionsList(planarRegionsList);

      PawPlanningResult result = planner.plan();

      if (activelyVisualize)
      {
         visualizer.showAndSleep(true);
      }

      assertTrue("Result was : " + result, result.validForExecution());
      PawPlan steps = planner.getPlan();

      if (visualize && !activelyVisualize)
         visualizePlan(steps, planarRegionsList, startPose.getPosition(), goalPose.getPosition());

      assertPlanIsValid(steps, goalPose.getPosition(), goalPose.getYaw());
   }

   private static void assertPlanIsValid(PawPlan plannedSteps, Point3DReadOnly goalPosition, double goalYaw)
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

      double centerDistance = goalPosition.distanceXY(centerPoint);
      double yawDistance = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(goalYaw, nominalYaw));
      assertTrue("Goal Position expected:\n" + goalPosition +"\n but was:\n" + centerPoint + "\nDifference of: " + centerDistance, centerDistance < PawNode.gridSizeXY);
      assertTrue("Tried to acheive yaw " + goalYaw + ", actually achieved " + nominalYaw, yawDistance < PawNode.gridSizeYaw);
   }

   private static QuadrantDependentList<Point3DBasics> getFinalStepPositions(PawPlan plannedSteps)
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

   private void visualizePlan(PawPlan steps, PlanarRegionsList planarRegionsList, Point3DReadOnly start, Point3DReadOnly goal)
   {
      if (!visualize || ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
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

      if (steps != null)
      {
         for (int i = 0; i < steps.getNumberOfSteps(); i++)
         {
            Point3DReadOnly point = steps.getPawStep(i).getGoalPosition();
            AppearanceDefinition appearanceDefinition = colorDefinitions.get(steps.getPawStep(i).getRobotQuadrant());

            graphics3DObject.identity();
            graphics3DObject.translate(point);
            graphics3DObject.addSphere(0.03, appearanceDefinition);

         }
      }

      scs.addStaticLinkGraphics(graphics3DObject);

      scs.setCameraFix(0.0, 0.0, 0.0);
      scs.setCameraPosition(-0.001, 0.0, 15.0);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }
}

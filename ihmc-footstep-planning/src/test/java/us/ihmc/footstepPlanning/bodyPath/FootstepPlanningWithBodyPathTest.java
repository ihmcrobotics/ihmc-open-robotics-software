package us.ihmc.footstepPlanning.bodyPath;

import java.util.ArrayList;
import java.util.List;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TestName;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FlatGroundFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.heuristics.BodyPathHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.AlwaysValidNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.DistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlan;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FootstepPlanningWithBodyPathTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getKeepSCSUp();

   @Rule
   public TestName name = new TestName();

   @Test(timeout = 30000)
   public void testWaypointPathOnFlat()
   {
      YoVariableRegistry registry = new YoVariableRegistry(name.getMethodName());
      FootstepPlannerParameters parameters = new DefaultFootstepPlanningParameters();
      double defaultStepWidth = parameters.getIdealFootstepWidth();

      double goalDistance = 5.0;
      FramePose initialStanceFootPose = new FramePose();
      RobotSide initialStanceFootSide = RobotSide.LEFT;
      initialStanceFootPose.setY(initialStanceFootSide.negateIfRightSide(defaultStepWidth / 2.0));
      FramePose goalPose = new FramePose();
      goalPose.setX(goalDistance);

      WaypointDefinedBodyPathPlan bodyPath = new WaypointDefinedBodyPathPlan();
      List<Point2D> waypoints = new ArrayList<>();
      waypoints.add(new Point2D(0.0, 0.0));
      waypoints.add(new Point2D(goalDistance / 8.0, 2.0));
      waypoints.add(new Point2D(2.0 * goalDistance / 3.0, -2.0));
      waypoints.add(new Point2D(7.0 * goalDistance / 8.0, -2.0));
      waypoints.add(new Point2D(goalDistance, 0.0));
      bodyPath.setWaypoints(waypoints);
      bodyPath.compute(null, null);

      FootstepPlanner planner = createBodyPathBasedPlanner(registry, parameters, bodyPath);
      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(planner, initialStanceFootPose, initialStanceFootSide, goalPose, null, true);

      if (visualize)
         PlanningTestTools.visualizeAndSleep(null, footstepPlan, goalPose, bodyPath);
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   public void testMaze()
   {
      WaypointDefinedBodyPathPlan bodyPath = new WaypointDefinedBodyPathPlan();
      List<Point2D> waypoints = new ArrayList<>();

      ArrayList<PlanarRegion> regions = PointCloudTools.loadPlanarRegionsFromFile("resources/PlanarRegions_NRI_Maze.txt");
      Point3D startPos = new Point3D(9.5, 9, 0);
      Point3D goalPos = new Point3D(0.5, 0.5, 0);
      startPos = PlanarRegionTools.projectPointToPlanes(startPos, new PlanarRegionsList(regions));
      goalPos = PlanarRegionTools.projectPointToPlanes(goalPos, new PlanarRegionsList(regions));

      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(new DefaultVisibilityGraphParameters());
      List<Point3DReadOnly> path = new ArrayList<>(navigableRegionsManager.calculateBodyPath(startPos, goalPos));
      for (Point3DReadOnly waypoint3d : path)
      {
         waypoints.add(new Point2D(waypoint3d.getX(), waypoint3d.getY()));
      }
      bodyPath.setWaypoints(waypoints);
      bodyPath.compute(null, null);

      Pose2D startPose = new Pose2D();
      bodyPath.getPointAlongPath(0.0, startPose);
      Pose2D finalPose = new Pose2D();
      bodyPath.getPointAlongPath(1.0, finalPose);

      YoVariableRegistry registry = new YoVariableRegistry(name.getMethodName());
      FootstepPlannerParameters parameters = new DefaultFootstepPlanningParameters();
      double defaultStepWidth = parameters.getIdealFootstepWidth();

      FramePose initialMidFootPose = new FramePose();
      initialMidFootPose.setX(startPos.getX());
      initialMidFootPose.setY(startPos.getY());
      initialMidFootPose.setYawPitchRoll(startPose.getYaw(), 0.0, 0.0);
      PoseReferenceFrame midFootFrame = new PoseReferenceFrame("InitialMidFootFrame", initialMidFootPose);

      RobotSide initialStanceFootSide = RobotSide.RIGHT;
      FramePose initialStanceFootPose = new FramePose(midFootFrame);
      initialStanceFootPose.setY(initialStanceFootSide.negateIfRightSide(defaultStepWidth / 2.0));
      initialStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose goalPose = new FramePose();
      goalPose.setX(finalPose.getX());
      goalPose.setY(finalPose.getY());
      goalPose.setYawPitchRoll(finalPose.getYaw(), 0.0, 0.0);

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(regions);
      AStarFootstepPlanner planner = createBodyPathBasedPlanner(registry, parameters, bodyPath);
      planner.setTimeout(1.0);
      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(planner, initialStanceFootPose, initialStanceFootSide, goalPose, planarRegionsList, true);

      if (visualize)
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose, bodyPath);
   }

   private AStarFootstepPlanner createBodyPathBasedPlanner(YoVariableRegistry registry, FootstepPlannerParameters parameters,
                                                           WaypointDefinedBodyPathPlan bodyPath)
   {
      FootstepNodeChecker nodeChecker = new AlwaysValidNodeChecker();
      CostToGoHeuristics heuristics = new BodyPathHeuristics(registry, parameters, bodyPath);
      FootstepNodeExpansion nodeExpansion = new ParameterBasedNodeExpansion(parameters);
      FootstepCost stepCostCalculator = new DistanceAndYawBasedCost(parameters);
      FlatGroundFootstepNodeSnapper snapper = new FlatGroundFootstepNodeSnapper();

      heuristics.setWeight(10.0);
      AStarFootstepPlanner planner = new AStarFootstepPlanner(parameters, nodeChecker, heuristics, nodeExpansion, stepCostCalculator, snapper, registry);
      return planner;
   }
}

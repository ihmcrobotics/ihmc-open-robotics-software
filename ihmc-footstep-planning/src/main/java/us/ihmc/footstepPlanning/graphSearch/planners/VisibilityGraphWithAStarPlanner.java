package us.ihmc.footstepPlanning.graphSearch.planners;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.heuristics.BodyPathHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.stepCost.DistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlan;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class VisibilityGraphWithAStarPlanner implements FootstepPlanner
{
   private static final double defaultHeuristicWeight = 3.0;
   private static final double planningHorizon = 2.0;

   private final FootstepPlannerParameters parameters;
   private final WaypointDefinedBodyPathPlan bodyPath;
   private final BodyPathHeuristics heuristics;
   private final FootstepPlanner footstepPlanner;

   private PlanarRegionsList planarRegionsList;
   private final FramePose bodyStartPose = new FramePose();
   private final FramePose bodyGoalPose = new FramePose();
   private final List<Point2D> waypoints = new ArrayList<>();

   public VisibilityGraphWithAStarPlanner(FootstepPlannerParameters parameters, SideDependentList<ConvexPolygon2D> footPolygons, YoVariableRegistry registry)
   {
      this.parameters = parameters;
      bodyPath = new WaypointDefinedBodyPathPlan();
      heuristics = new BodyPathHeuristics(registry, parameters, bodyPath);

      FootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      FootstepNodeChecker nodeChecker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
      FootstepCost stepCostCalculator = new DistanceAndYawBasedCost(parameters);
      FootstepNodeSnapper postProcessingSnapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters, null);

      heuristics.setWeight(defaultHeuristicWeight);
      footstepPlanner = new AStarFootstepPlanner(parameters, nodeChecker, heuristics, expansion, stepCostCalculator, postProcessingSnapper, registry);
   }

   @Override
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide side)
   {
      double defaultStepWidth = parameters.getIdealFootstepWidth();
      ReferenceFrame stanceFrame = new PoseReferenceFrame("stanceFrame", stanceFootPose);
      bodyStartPose.setToZero(stanceFrame);
      bodyStartPose.setY(side.negateIfLeftSide(defaultStepWidth / 2.0));
      bodyStartPose.changeFrame(ReferenceFrame.getWorldFrame());

      footstepPlanner.setInitialStanceFoot(stanceFootPose, side);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      AStarFootstepPlanner.checkGoalType(goal);
      bodyGoalPose.setIncludingFrame(goal.getGoalPoseBetweenFeet());
   }

   @Override
   public void setTimeout(double timeout)
   {
      // only considers footstep planner right now
      footstepPlanner.setTimeout(timeout);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      footstepPlanner.setPlanarRegions(planarRegionsList);
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public FootstepPlanningResult plan()
   {
      throw new IllegalAccessError("Fix PlanarRegionTools import issue");
//      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(planarRegionsList.getPlanarRegionsAsList());
//      Point3D startPos = PlanarRegionTools.projectPointToPlanesVertically(bodyStartPose.getPosition(), planarRegionsList);
//      Point3D goalPos = PlanarRegionTools.projectPointToPlanesVertically(bodyGoalPose.getPosition(), planarRegionsList);
//      ArrayList<Point3D> path = navigableRegionsManager.calculateBodyPath(startPos, goalPos);
//
//      waypoints.clear();
//      for (Point3D waypoint3d : path)
//      {
//         waypoints.add(new Point2D(waypoint3d.getX(), waypoint3d.getY()));
//      }
//      bodyPath.setWaypoints(waypoints);
//      bodyPath.compute(null, null);
//
//      Pose2D goalPose2d = new Pose2D();
//      double pathLength = bodyPath.computePathLength(0.0);
//      double alpha = MathTools.clamp(planningHorizon / pathLength, 0.0, 1.0);
//      bodyPath.getPointAlongPath(alpha, goalPose2d);
//      heuristics.setGoalAlpha(alpha);
//
//      FramePose footstepPlannerGoal = new FramePose();
//      footstepPlannerGoal.setPosition(goalPose2d.getX(), goalPose2d.getY(), 0.0);
//      footstepPlannerGoal.setYawPitchRoll(goalPose2d.getYaw(), 0.0, 0.0);
//
//      FootstepPlannerGoal goal = new FootstepPlannerGoal();
//      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
//      goal.setGoalPoseBetweenFeet(footstepPlannerGoal);
//      footstepPlanner.setGoal(goal);
//
//      return footstepPlanner.plan();
   }

   @Override
   public FootstepPlan getPlan()
   {
      return footstepPlanner.getPlan();
   }

}

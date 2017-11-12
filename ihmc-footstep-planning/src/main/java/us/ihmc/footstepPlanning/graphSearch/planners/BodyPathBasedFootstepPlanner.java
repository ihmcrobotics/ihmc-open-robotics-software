package us.ihmc.footstepPlanning.graphSearch.planners;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
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
import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.stepCost.DistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlan;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class BodyPathBasedFootstepPlanner implements FootstepPlanner
{
   private final FootstepPlannerParameters parameters;
   private final WaypointDefinedBodyPathPlan bodyPath;
   private final FootstepPlanner footstepPlanner;

   private final FramePose bodyStart = new FramePose();
   private final FramePose bodyGoal = new FramePose();

   private static final int numberOfPoints = 5;
   private final List<Point2D> waypoints = new ArrayList<>();
   private final YoPolynomial xPoly;
   private final YoPolynomial yPoly;

   private static final double weight = 1.0;
   private static final double horizon = 1.0;

   public BodyPathBasedFootstepPlanner(FootstepPlannerParameters parameters, SideDependentList<ConvexPolygon2D> footPolygons, YoVariableRegistry registry)
   {
      this.parameters = parameters;
      xPoly = new YoPolynomial("xPoly", 4, registry);
      yPoly = new YoPolynomial("yPoly", 4, registry);

      bodyPath = new WaypointDefinedBodyPathPlan();

      FootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      FootstepNodeChecker nodeChecker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);
      CostToGoHeuristics heuristics = new BodyPathHeuristics(registry, parameters, bodyPath);
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
      FootstepCost stepCostCalculator = new DistanceAndYawBasedCost(parameters);
      FootstepNodeSnapper postProcessingSnapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters, null);

      heuristics.setWeight(weight);
      footstepPlanner = new AStarFootstepPlanner(parameters, nodeChecker, heuristics, expansion, stepCostCalculator, postProcessingSnapper, registry);
   }

   @Override
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide side)
   {
      double defaultStepWidth = parameters.getIdealFootstepWidth();
      ReferenceFrame stanceFrame = new PoseReferenceFrame("stanceFrame", stanceFootPose);
      FramePoint2D bodyStartPoint = new FramePoint2D(stanceFrame);
      bodyStartPoint.setY(side.negateIfLeftSide(defaultStepWidth / 2.0));
      bodyStartPoint.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      bodyStart.setToZero(ReferenceFrame.getWorldFrame());
      bodyStart.setPosition(bodyStartPoint.getX(), bodyStartPoint.getY(), 0.0);
      bodyStart.setYawPitchRoll(stanceFootPose.getYaw(), 0.0, 0.0);

      footstepPlanner.setInitialStanceFoot(stanceFootPose, side);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      AStarFootstepPlanner.checkGoalType(goal);
      FramePose goalPose = goal.getGoalPoseBetweenFeet();
      bodyGoal.setIncludingFrame(goalPose);
   }

   @Override
   public void setTimeout(double timeout)
   {
      // assumes very fast body path planner
      footstepPlanner.setTimeout(timeout);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      bodyPath.setPlanarRegionsList(planarRegionsList);
      footstepPlanner.setPlanarRegions(planarRegionsList);
   }

   @Override
   public FootstepPlanningResult plan()
   {
      waypoints.clear();
      double yaw = bodyStart.getYaw();
      xPoly.setQuadratic(0.0, 1.0, bodyStart.getX(), Math.cos(yaw) * 0.2, bodyGoal.getX());
      yPoly.setQuadratic(0.0, 1.0, bodyStart.getY(), Math.sin(yaw) * 0.2, bodyGoal.getY());
      for (int i = 0; i < numberOfPoints; i++)
      {
         double percent = i / (double) (numberOfPoints - 1);
         xPoly.compute(percent);
         yPoly.compute(percent);
         Point2D point2d = new Point2D(xPoly.getPosition(), yPoly.getPosition());
         waypoints.add(point2d);
      }
      bodyPath.setWaypoints(waypoints);
      bodyPath.compute(null, null);

      Pose2D goalPose2d = new Pose2D();
      double pathLength = bodyPath.computePathLength(0.0);
      double alpha = MathTools.clamp(horizon / pathLength, 0.0, 1.0);
      bodyPath.getPointAlongPath(alpha, goalPose2d);

      FramePose footstepPlannerGoal = new FramePose();
      footstepPlannerGoal.setPosition(goalPose2d.getX(), goalPose2d.getY(), 0.0);
      footstepPlannerGoal.setYawPitchRoll(goalPose2d.getYaw(), 0.0, 0.0);

      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(footstepPlannerGoal);
      footstepPlanner.setGoal(goal);

      return footstepPlanner.plan();
   }

   @Override
   public FootstepPlan getPlan()
   {
      return footstepPlanner.getPlan();
   }
}

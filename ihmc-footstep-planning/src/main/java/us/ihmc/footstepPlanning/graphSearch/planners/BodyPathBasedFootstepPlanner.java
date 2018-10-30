package us.ihmc.footstepPlanning.graphSearch.planners;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.heuristics.BodyPathHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.stepCost.EuclideanDistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostBuilder;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanner;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BodyPathBasedFootstepPlanner implements FootstepPlanner
{
   private static final boolean debug = false;
   private static final RobotSide defaultStartNodeSide = RobotSide.LEFT;

   private final FootstepPlannerParameters parameters;
   private final WaypointDefinedBodyPathPlanner bodyPath;
   private final FootstepPlanner footstepPlanner;

   private final FramePose3D bodyStart = new FramePose3D();
   private final FramePose3D bodyGoal = new FramePose3D();

   private static final int numberOfPoints = 5;
   private final List<Point3D> waypoints = new ArrayList<>();
   private final YoPolynomial xPoly;
   private final YoPolynomial yPoly;
   private final YoPolynomial zPoly;

   private final YoDouble planningHorizonLength;

   private static final double weight = 1.0;

   public BodyPathBasedFootstepPlanner(FootstepPlannerParameters parameters, SideDependentList<ConvexPolygon2D> footPolygons, YoVariableRegistry registry)
   {
      this.parameters = parameters;
      xPoly = new YoPolynomial("xPoly", 4, registry);
      yPoly = new YoPolynomial("yPoly", 4, registry);
      zPoly = new YoPolynomial("zPoly", 4, registry);

      bodyPath = new WaypointDefinedBodyPathPlanner();

      FootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons, parameters);
      FootstepNodeChecker nodeChecker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);
      CostToGoHeuristics heuristics = new BodyPathHeuristics(parameters.getCostParameters().getBodyPathBasedHeuristicsWeight(), parameters, bodyPath);
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);

      FootstepCostBuilder costBuilder = new FootstepCostBuilder();
      costBuilder.setFootstepPlannerParameters(parameters);
      costBuilder.setSnapper(snapper);
      costBuilder.setIncludeHeightCost(false);
      costBuilder.setIncludePitchAndRollCost(false);

      FootstepCost footstepCost = costBuilder.buildCost();

      FootstepNodeSnapper postProcessingSnapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters, null);

      footstepPlanner = new AStarFootstepPlanner(parameters, nodeChecker, heuristics, expansion, footstepCost, postProcessingSnapper, registry);

      planningHorizonLength = new YoDouble("planningHorizonLength", registry);
      planningHorizonLength.set(1.0);
   }

   @Override
   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side)
   {
      if (side == null)
      {
         if (debug)
            PrintTools.info("Start node needs a side, but trying to set it to null. Setting it to " + defaultStartNodeSide);

         side = defaultStartNodeSide;
      }

      double defaultStepWidth = parameters.getIdealFootstepWidth();
      ReferenceFrame stanceFrame = new PoseReferenceFrame("stanceFrame", stanceFootPose);
      FramePoint2D bodyStartPoint = new FramePoint2D(stanceFrame);
      bodyStartPoint.setY(side.negateIfLeftSide(defaultStepWidth / 2.0));
      bodyStartPoint.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      bodyStart.setToZero(ReferenceFrame.getWorldFrame());
      bodyStart.setPosition(bodyStartPoint.getX(), bodyStartPoint.getY(), 0.0);
      bodyStart.setOrientationYawPitchRoll(stanceFootPose.getYaw(), 0.0, 0.0);

      footstepPlanner.setInitialStanceFoot(stanceFootPose, side);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      AStarFootstepPlanner.checkGoalType(goal);
      FramePose3D goalPose = goal.getGoalPoseBetweenFeet();
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
   public double getPlanningDuration()
   {
      return footstepPlanner.getPlanningDuration();
   }

   @Override
   public void setPlanningHorizonLength(double planningHorizonLength)
   {
      this.planningHorizonLength.set(planningHorizonLength);
   }

   @Override
   public FootstepPlanningResult plan()
   {
      waypoints.clear();
      double yaw = bodyStart.getYaw();
      xPoly.setQuadratic(0.0, 1.0, bodyStart.getX(), Math.cos(yaw) * 0.2, bodyGoal.getX());
      yPoly.setQuadratic(0.0, 1.0, bodyStart.getY(), Math.sin(yaw) * 0.2, bodyGoal.getY());
      zPoly.setQuadratic(0.0, 1.0, bodyStart.getZ(), Math.sin(yaw) * 0.2, bodyGoal.getZ());
      for (int i = 0; i < numberOfPoints; i++)
      {
         double percent = i / (double) (numberOfPoints - 1);
         xPoly.compute(percent);
         yPoly.compute(percent);
         zPoly.compute(percent);
         Point3D point = new Point3D(xPoly.getPosition(), yPoly.getPosition(), zPoly.getPosition());
         waypoints.add(point);
      }
      bodyPath.setWaypoints(waypoints);
      bodyPath.compute();

      Pose2D goalPose2d = new Pose2D();
      double pathLength = bodyPath.computePathLength(0.0);
      double alpha = MathTools.clamp(planningHorizonLength.getDoubleValue() / pathLength, 0.0, 1.0);
      bodyPath.getPointAlongPath(alpha, goalPose2d);

      FramePose3D footstepPlannerGoal = new FramePose3D();
      footstepPlannerGoal.setPosition(goalPose2d.getX(), goalPose2d.getY(), 0.0);
      footstepPlannerGoal.setOrientationYawPitchRoll(goalPose2d.getYaw(), 0.0, 0.0);

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

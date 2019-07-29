package us.ihmc.footstepPlanning.graphSearch.planners;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.heuristics.BodyPathHeuristics;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostBuilder;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BodyPathBasedAStarPlanner implements FootstepPlanner
{
   private static final boolean debug = false;
   private static final RobotSide defaultStartNodeSide = RobotSide.LEFT;

   private final BodyPathPlanner bodyPathPlanner;
   private final FootstepPlanner footstepPlanner;

   private final BodyPathHeuristics heuristics;
   private final YoDouble planningHorizonLength;

   public BodyPathBasedAStarPlanner(String prefix, BodyPathPlanner bodyPathPlanner, FootstepPlannerParametersReadOnly parameters,
                                    SideDependentList<ConvexPolygon2D> footPolygons, DoubleProvider heuristicWeight, YoVariableRegistry parentRegistry,
                                    BipedalFootstepPlannerListener... listeners)
   {
      this.bodyPathPlanner = bodyPathPlanner;

      YoVariableRegistry registry = new YoVariableRegistry(prefix + getClass().getSimpleName());


      heuristics = new BodyPathHeuristics(heuristicWeight, parameters, this.bodyPathPlanner);

      FootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      FootstepNodeChecker nodeChecker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
      FootstepNodeSnapper postProcessingSnapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);

      FootstepCostBuilder costBuilder = new FootstepCostBuilder();
      costBuilder.setFootstepPlannerParameters(parameters);
      costBuilder.setSnapper(snapper);
      costBuilder.setIncludeHeightCost(false);
      costBuilder.setIncludePitchAndRollCost(false);

      FootstepCost footstepCost = costBuilder.buildCost();

      planningHorizonLength = new YoDouble("planningHorizonLength", registry);
      planningHorizonLength.set(1.0);

      for (BipedalFootstepPlannerListener listener : listeners)
      {
         nodeChecker.addPlannerListener(listener);
      }

      if (listeners.length > 0)
      {
         footstepPlanner = new AStarFootstepPlanner(parameters, nodeChecker, heuristics, expansion, footstepCost, postProcessingSnapper, listeners[0], null,
                                                    registry);
      }
      else
      {
         footstepPlanner = new AStarFootstepPlanner(parameters, nodeChecker, heuristics, expansion, footstepCost, postProcessingSnapper, registry);
      }

      parentRegistry.addChild(registry);
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

      footstepPlanner.setInitialStanceFoot(stanceFootPose, side);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
   }

   @Override
   public void setTimeout(double timeout)
   {
      footstepPlanner.setTimeout(timeout);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
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
   public double getPlanningHorizonLength()
   {
      return planningHorizonLength.getDoubleValue();
   }

   @Override
   public FootstepPlanningResult plan()
   {
      Pose2D goalPose2d = new Pose2D();
      double pathLength = bodyPathPlanner.computePathLength(0.0);
      double alpha = MathTools.clamp(planningHorizonLength.getDoubleValue() / pathLength, 0.0, 1.0);
      bodyPathPlanner.getPointAlongPath(alpha, goalPose2d);
      heuristics.setGoalAlpha(alpha);

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
   public void cancelPlanning()
   {
      footstepPlanner.cancelPlanning();
   }

   @Override
   public FootstepPlan getPlan()
   {
      return footstepPlanner.getPlan();
   }
}

package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch;

import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;

import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.SimplePlanarRegionPawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics.*;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking.*;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeExpansion.PawNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeExpansion.ParameterBasedPawNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost.PawNodeCost;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost.PawNodeCostBuilder;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Arrays;

public class BodyPathBasedAStarPawPlanner implements PawPlanner
{
   private final BodyPathPlanner bodyPathPlanner;
   private final PawPlanner pawPlanner;

   private final FramePose3D lowLevelGoal = new FramePose3D();

   private final BodyPathPawPlanningHeuristics bodyPathHeuristics;
   private final PawPlanningCostToGoHeuristics heuristics;
   private final YoDouble planningHorizonLength;

   private PawPlannerGoal highLevelGoal;

   public BodyPathBasedAStarPawPlanner(String prefix, BodyPathPlanner bodyPathPlanner, PawPlannerParameters parameters,
                                       QuadrupedXGaitSettingsReadOnly xGaitSettings, YoVariableRegistry parentRegistry)
   {
      this.bodyPathPlanner = bodyPathPlanner;

      YoVariableRegistry registry = new YoVariableRegistry(prefix + getClass().getSimpleName());

      PawNodeSnapper snapper = new SimplePlanarRegionPawNodeSnapper(parameters, parameters::getProjectInsideDistance,
                                                                    parameters::getProjectInsideUsingConvexHull, true);

      CompositePawPlanningCostToGoHeuristics costToGoHeuristics = new CompositePawPlanningCostToGoHeuristics(parameters);

      bodyPathHeuristics = new BodyPathPawPlanningHeuristics(parameters, this.bodyPathPlanner, xGaitSettings, snapper);

      costToGoHeuristics.addCostToGoHeuristic(bodyPathHeuristics);

      heuristics = costToGoHeuristics;

      PawNodeTransitionChecker snapBasedNodeTransitionChecker = new SnapBasedPawNodeTransitionChecker(parameters, snapper);
      PawNodeChecker snapBasedNodeChecker = new SnapBasedPawNodeChecker(parameters, snapper);
      PawPlanarRegionCliffAvoider cliffAvoider = new PawPlanarRegionCliffAvoider(parameters, snapper);
      PawNodeTransitionChecker nodeTransitionChecker = new PawNodeTransitionCheckerOfCheckers(Arrays.asList(snapBasedNodeTransitionChecker));
      PawNodeChecker nodeChecker = new PawNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker, cliffAvoider));

      PawNodeExpansion expansion = new ParameterBasedPawNodeExpansion(parameters, xGaitSettings);

      PawNodeCostBuilder costBuilder = new PawNodeCostBuilder();
      costBuilder.setPawPlannerParameters(parameters);
      costBuilder.setXGaitSettings(xGaitSettings);
      costBuilder.setSnapper(snapper);
      costBuilder.setIncludeHeightCost(true);

      PawNodeCost pawNodeCost = costBuilder.buildCost();

      planningHorizonLength = new YoDouble("planningHorizonLength", registry);
      planningHorizonLength.set(1.0);

      pawPlanner = new AStarPawPlanner(parameters, xGaitSettings, nodeChecker, nodeTransitionChecker, heuristics, expansion, pawNodeCost,
                                       snapper, snapper, null, registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void setStart(PawPlannerStart start)
   {
      pawPlanner.setStart(start);
   }

   @Override
   public void setGoal(PawPlannerGoal goal)
   {
      highLevelGoal = goal;
   }

   @Override
   public void setTimeout(double timeout)
   {
      pawPlanner.setTimeout(timeout);
   }

   @Override
   public void setBestEffortTimeout(double bestEffortTimeout)
   {
      pawPlanner.setBestEffortTimeout(bestEffortTimeout);
   }

   @Override
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      pawPlanner.setPlanarRegionsList(planarRegionsList);
   }

   @Override
   public void setGroundPlane(QuadrupedGroundPlaneMessage groundPlane)
   {

   }

   @Override
   public double getPlanningDuration()
   {
      return pawPlanner.getPlanningDuration();
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
   public PawPlanningResult plan()
   {
      Pose2D goalPose2d = new Pose2D();
      double pathLength = bodyPathPlanner.computePathLength(0.0);
      double alpha = MathTools.clamp(planningHorizonLength.getDoubleValue() / pathLength, 0.0, 1.0);
      bodyPathPlanner.getPointAlongPath(alpha, goalPose2d);
      bodyPathHeuristics.setGoalAlpha(alpha);

      FramePose3D pawPlannerGoal = new FramePose3D();
      pawPlannerGoal.setPosition(goalPose2d.getX(), goalPose2d.getY(), 0.0);
      pawPlannerGoal.setOrientationYawPitchRoll(goalPose2d.getYaw(), 0.0, 0.0);

      if (alpha >= 1.0)
         pawPlannerGoal.setOrientation(highLevelGoal.getTargetPose().getOrientation());

      lowLevelGoal.set(pawPlannerGoal);

      PawPlannerGoal goal = new PawPlannerGoal();
      goal.setGoalPose(pawPlannerGoal);
      goal.setGoalType(PawPlannerTargetType.POSE_BETWEEN_FEET);
      pawPlanner.setGoal(goal);

      return pawPlanner.plan();
   }

   @Override
   public void cancelPlanning()
   {
      pawPlanner.cancelPlanning();
   }

   @Override
   public PawPlan getPlan()
   {
      PawPlan pawPlan = pawPlanner.getPlan();
      if (pawPlan != null)
         pawPlan.setLowLevelPlanGoal(lowLevelGoal);

      return pawPlan;
   }
}

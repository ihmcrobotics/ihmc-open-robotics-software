package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch;

import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;

import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanHolder;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.SimplePlanarRegionPawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics.*;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking.*;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeExpansion.PawNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeExpansion.ParameterBasedPawNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost.PawNodeCost;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost.PawNodeCostBuilder;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Arrays;

public class BodyPathBasedAStarPawPlanner implements PawStepPlanner
{
   private final BodyPathPlanHolder bodyPathPlanner;
   private final PawStepPlanner pawStepPlanner;

   private final FramePose3D lowLevelGoal = new FramePose3D();

   private final BodyPathPawPlanningHeuristics bodyPathHeuristics;
   private final PawPlanningCostToGoHeuristics heuristics;
   private final YoDouble planningHorizonLength;

   private PawStepPlannerGoal highLevelGoal;

   public BodyPathBasedAStarPawPlanner(String prefix, BodyPathPlanHolder bodyPathPlanner, PawStepPlannerParametersReadOnly parameters,
                                       QuadrupedXGaitSettingsReadOnly xGaitSettings, YoVariableRegistry parentRegistry)
   {
      this.bodyPathPlanner = bodyPathPlanner;

      YoVariableRegistry registry = new YoVariableRegistry(prefix + getClass().getSimpleName());

      PawNodeSnapper snapper = new SimplePlanarRegionPawNodeSnapper(parameters, true);
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

      pawStepPlanner = new AStarPawStepPlanner(parameters, xGaitSettings, nodeChecker, nodeTransitionChecker, heuristics, expansion, pawNodeCost,
                                               snapper, null, registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void setStart(PawStepPlannerStart start)
   {
      pawStepPlanner.setStart(start);
   }

   @Override
   public void setGoal(PawStepPlannerGoal goal)
   {
      highLevelGoal = goal;
   }

   @Override
   public void setTimeout(double timeout)
   {
      pawStepPlanner.setTimeout(timeout);
   }

   @Override
   public void setBestEffortTimeout(double bestEffortTimeout)
   {
      pawStepPlanner.setBestEffortTimeout(bestEffortTimeout);
   }

   @Override
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      pawStepPlanner.setPlanarRegionsList(planarRegionsList);
   }

   @Override
   public void setGroundPlane(QuadrupedGroundPlaneMessage groundPlane)
   {

   }

   @Override
   public double getPlanningDuration()
   {
      return pawStepPlanner.getPlanningDuration();
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
   public PawStepPlanningResult plan()
   {
      Pose3D goalPose = new Pose3D();
      double pathLength = bodyPathPlanner.computePathLength(0.0);
      double alpha = MathTools.clamp(planningHorizonLength.getDoubleValue() / pathLength, 0.0, 1.0);
      bodyPathPlanner.getPointAlongPath(alpha, goalPose);
      bodyPathHeuristics.setGoalAlpha(alpha);

      FramePose3D pawPlannerGoal = new FramePose3D();
      pawPlannerGoal.set(goalPose);
//      pawPlannerGoal.setPosition(goalPose.getPosition());
//      pawPlannerGoal.setOrientationYawPitchRoll(goalPose2d.getYaw(), 0.0, 0.0);

      if (alpha >= 1.0)
         pawPlannerGoal.setOrientation(highLevelGoal.getTargetPose().getOrientation());

      lowLevelGoal.set(pawPlannerGoal);

      PawStepPlannerGoal goal = new PawStepPlannerGoal();
      goal.setGoalPose(pawPlannerGoal);
      goal.setGoalType(PawStepPlannerTargetType.POSE_BETWEEN_FEET);
      pawStepPlanner.setGoal(goal);

      return pawStepPlanner.plan();
   }

   @Override
   public void cancelPlanning()
   {
      pawStepPlanner.cancelPlanning();
   }

   @Override
   public PawStepPlan getPlan()
   {
      PawStepPlan pawStepPlan = pawStepPlanner.getPlan();
      if (pawStepPlan != null)
         pawStepPlan.setLowLevelPlanGoal(lowLevelGoal);

      return pawStepPlan;
   }
}

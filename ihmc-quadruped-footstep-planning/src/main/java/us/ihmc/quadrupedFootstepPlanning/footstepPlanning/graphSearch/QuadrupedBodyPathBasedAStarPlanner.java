package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch;

import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;

import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodePlanarRegionSnapAndWiggler;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics.BodyPathHeuristics;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.FootstepNodeCheckerOfCheckers;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.PlanarRegionCliffAvoider;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.VariableResolutionNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost.BodyPathBasedVelocityProvider;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost.FootstepCostBuilder;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost.NominalVelocityProvider;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Arrays;

public class QuadrupedBodyPathBasedAStarPlanner implements QuadrupedFootstepPlanner
{
   private static final boolean debug = false;
   private static final RobotSide defaultStartNodeSide = RobotSide.LEFT;

   private final BodyPathPlanner bodyPathPlanner;
   private final QuadrupedFootstepPlanner footstepPlanner;

   private final FramePose3D lowLevelGoal = new FramePose3D();

   private final BodyPathHeuristics heuristics;
   private final YoDouble planningHorizonLength;

   public QuadrupedBodyPathBasedAStarPlanner(String prefix, BodyPathPlanner bodyPathPlanner, FootstepPlannerParameters parameters,
                                             QuadrupedXGaitSettingsReadOnly xGaitSettings, YoVariableRegistry parentRegistry)
   {
      this.bodyPathPlanner = bodyPathPlanner;

      YoVariableRegistry registry = new YoVariableRegistry(prefix + getClass().getSimpleName());


      NominalVelocityProvider velocityProvider = new BodyPathBasedVelocityProvider(this.bodyPathPlanner);

      FootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters, parameters::getProjectInsideDistanceForExpansion,
                                                                              parameters::getProjectInsideUsingConvexHullDuringExpansion, true);
      heuristics = new BodyPathHeuristics(parameters, this.bodyPathPlanner, xGaitSettings, snapper);

      FootstepNodeChecker snapBasedNodeChecker = new SnapBasedNodeChecker(parameters, snapper);
      PlanarRegionCliffAvoider cliffAvoider = new PlanarRegionCliffAvoider(parameters, snapper);
      FootstepNodeChecker nodeChecker = new FootstepNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker, cliffAvoider));

      FootstepNodeExpansion expansion = new VariableResolutionNodeExpansion(parameters, xGaitSettings, snapper);
      FootstepNodeSnapper postProcessingSnapper = new FootstepNodePlanarRegionSnapAndWiggler(parameters, parameters::getProjectInsideDistanceForPostProcessing,
                                                                                             parameters::getProjectInsideUsingConvexHullDuringPostProcessing, false);

      FootstepCostBuilder costBuilder = new FootstepCostBuilder();
      costBuilder.setFootstepPlannerParameters(parameters);
      costBuilder.setXGaitSettings(xGaitSettings);
      costBuilder.setSnapper(snapper);
      costBuilder.setIncludeHeightCost(true);
      costBuilder.setVelocityProvider(velocityProvider);

      FootstepCost footstepCost = costBuilder.buildCost();

      planningHorizonLength = new YoDouble("planningHorizonLength", registry);
      planningHorizonLength.set(1.0);

      footstepPlanner = new QuadrupedAStarFootstepPlanner(parameters, xGaitSettings, nodeChecker, heuristics, velocityProvider, expansion, footstepCost, snapper,
                                                          postProcessingSnapper, null, registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void setStart(QuadrupedFootstepPlannerStart start)
   {
      footstepPlanner.setStart(start);
   }

   @Override
   public void setGoal(QuadrupedFootstepPlannerGoal goal)
   {
   }

   @Override
   public void setTimeout(double timeout)
   {
      footstepPlanner.setTimeout(timeout);
   }

   @Override
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      footstepPlanner.setPlanarRegionsList(planarRegionsList);
   }

   @Override
   public void setGroundPlane(QuadrupedGroundPlaneMessage groundPlane)
   {

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

      lowLevelGoal.set(footstepPlannerGoal);

      QuadrupedFootstepPlannerGoal goal = new QuadrupedFootstepPlannerGoal();
      goal.setGoalPose(footstepPlannerGoal);
      goal.setGoalType(FootstepPlannerTargetType.POSE_BETWEEN_FEET);
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
      FootstepPlan footstepPlan = footstepPlanner.getPlan();
      if (footstepPlan != null)
         footstepPlan.setLowLevelPlanGoal(lowLevelGoal);

      return footstepPlan;
   }
}

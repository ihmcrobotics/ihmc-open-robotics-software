package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class DistanceAndYawBasedHeuristics extends CostToGoHeuristics
{
   private final FootstepPlannerParametersReadOnly parameters;

   public DistanceAndYawBasedHeuristics(FootstepNodeSnapperReadOnly snapper, DoubleProvider weight, FootstepPlannerParametersReadOnly parameters)
   {
      super(weight, parameters, snapper);
      this.parameters = parameters;
   }

   @Override
   protected double computeHeuristics(FramePose3DReadOnly pose)
   {
      double euclideanDistance = pose.getPosition().distanceXY(goalPose.getPosition());

      double referenceYaw = computeReferenceYaw(pose, goalPose);
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(pose.getYaw(), referenceYaw);

      double heightCost;

      // add a two times multiplier because both feet have to move
      double heightChange = goalPose.getZ() - pose.getZ();
      if (heightChange > 0)
         heightCost = parameters.getStepUpWeight() * 2.0 * heightChange;
      else
         heightCost = -parameters.getStepDownWeight() * 2.0 * heightChange;

      double minSteps = euclideanDistance / parameters.getMaximumStepReach() + Math.abs(yaw) / (0.5 * parameters.getMaximumStepYaw());
      return euclideanDistance + parameters.getYawWeight() * Math.abs(yaw) + heightCost + parameters.getCostPerStep() * minSteps;
   }

   private double computeReferenceYaw(FramePose3DReadOnly pose, FramePose3D goalPose)
   {
      double distanceToGoal = pose.getPosition().distanceXY(goalPose.getPosition());
      double finalTurnProximity = parameters.getFinalTurnProximity();

      double minimumBlendDistance = (1.0 - parameters.getFinalTurnProximityBlendFactor()) * finalTurnProximity;
      double maximumBlendDistance = (1.0 + parameters.getFinalTurnProximityBlendFactor()) * finalTurnProximity;

      double pathHeading = BodyPathPlannerTools.calculateHeading(goalPose.getX() - pose.getX(), goalPose.getY() - pose.getY());

      double yawMultiplier;
      if (distanceToGoal < minimumBlendDistance)
         yawMultiplier = 0.0;
      else if (distanceToGoal > maximumBlendDistance)
         yawMultiplier = 1.0;
      else
         yawMultiplier = (distanceToGoal - minimumBlendDistance) / (maximumBlendDistance - minimumBlendDistance);

      return AngleTools.interpolateAngle(goalPose.getYaw(), pathHeading, yawMultiplier);
   }
}

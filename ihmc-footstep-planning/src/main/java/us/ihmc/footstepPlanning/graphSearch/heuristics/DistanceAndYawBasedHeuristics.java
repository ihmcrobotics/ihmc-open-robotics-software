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

import java.util.function.DoubleSupplier;

public class DistanceAndYawBasedHeuristics extends CostToGoHeuristics
{
   private final DoubleSupplier stepUpWeight;
   private final DoubleSupplier stepDownWeight;
   private final DoubleSupplier maximumStepReach;
   private final DoubleSupplier maximumStepYaw;
   private final DoubleSupplier yawWeight;
   private final DoubleSupplier costPerStep;
   private final DoubleSupplier finalTurnProximity;
   private final DoubleSupplier finalTurnProximityBlendFactor;

   public DistanceAndYawBasedHeuristics(FootstepNodeSnapperReadOnly snapper, DoubleProvider weight, FootstepPlannerParametersReadOnly parameters)
   {
      this(parameters::getStepUpWeight,
           parameters::getStepDownWeight,
           parameters::getMaximumStepReach,
           parameters::getMaximumStepYaw,
           parameters::getYawWeight,
           parameters::getCostPerStep,
           parameters::getFinalTurnProximity,
           parameters::getFinalTurnProximityBlendFactor,
           parameters::getIdealFootstepWidth,
           weight::getValue,
           snapper);
   }

   public DistanceAndYawBasedHeuristics(DoubleSupplier stepUpWeight,
                                        DoubleSupplier stepDownWeight,
                                        DoubleSupplier maximumStepReach,
                                        DoubleSupplier maximumStepYaw,
                                        DoubleSupplier yawWeight,
                                        DoubleSupplier costPerStep,
                                        DoubleSupplier finalTurnProximity,
                                        DoubleSupplier finalTurnProximityBlendFactor,
                                        DoubleSupplier idealFootstepWidth,
                                        DoubleSupplier weight,
                                        FootstepNodeSnapperReadOnly snapper)
   {
      super(weight, idealFootstepWidth, snapper);
      this.stepUpWeight = stepUpWeight;
      this.stepDownWeight = stepDownWeight;
      this.maximumStepReach = maximumStepReach;
      this.maximumStepYaw = maximumStepYaw;
      this.yawWeight = yawWeight;
      this.costPerStep = costPerStep;
      this.finalTurnProximity = finalTurnProximity;
      this.finalTurnProximityBlendFactor = finalTurnProximityBlendFactor;
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
         heightCost = stepUpWeight.getAsDouble() * 2.0 * heightChange;
      else
         heightCost = -stepDownWeight.getAsDouble() * 2.0 * heightChange;

      double minSteps = euclideanDistance / maximumStepReach.getAsDouble() + Math.abs(yaw) / (0.5 * maximumStepYaw.getAsDouble());
      return euclideanDistance + yawWeight.getAsDouble() * Math.abs(yaw) + heightCost + costPerStep.getAsDouble() * minSteps;
   }

   private double computeReferenceYaw(FramePose3DReadOnly pose, FramePose3D goalPose)
   {
      double distanceToGoal = pose.getPosition().distanceXY(goalPose.getPosition());
      double finalTurnProximity = this.finalTurnProximity.getAsDouble();

      double minimumBlendDistance = (1.0 - finalTurnProximityBlendFactor.getAsDouble()) * finalTurnProximity;
      double maximumBlendDistance = (1.0 + finalTurnProximityBlendFactor.getAsDouble()) * finalTurnProximity;

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

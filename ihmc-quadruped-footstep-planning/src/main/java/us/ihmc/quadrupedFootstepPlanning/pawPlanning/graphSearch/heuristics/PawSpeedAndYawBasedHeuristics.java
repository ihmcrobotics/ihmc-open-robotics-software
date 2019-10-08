package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.PawNodeCostTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNodeTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PawSpeedAndYawBasedHeuristics extends PawPlanningCostToGoHeuristics
{
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final PawNodeSnapper snapper;

   public PawSpeedAndYawBasedHeuristics(PawNodeSnapper snapper, PawStepPlannerParametersReadOnly parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      super(parameters);

      this.snapper = snapper;
      this.xGaitSettings = xGaitSettings;
   }

   @Override
   protected double computeHeuristics(PawNode node)
   {
      double bodyDistance = node.getOrComputeXGaitCenterPoint().distanceXY(goalPose.getPosition());
      double desiredSpeed = parameters.getMaxWalkingSpeedMultiplier() * xGaitSettings.getMaxSpeed();
      double minSteps = bodyDistance / desiredSpeed;

      double referenceYaw = PawNodeCostTools.computeReferenceYaw(node.getOrComputeXGaitCenterPoint(), node.getStepYaw(), goalPose, parameters.getFinalTurnProximity());
      double angleDifference = AngleTools.computeAngleDifferenceMinusPiToPi(node.getStepYaw(), referenceYaw);
      double yawHeuristicCost = parameters.getYawWeight() * Math.abs(angleDifference);

      double stepHeuristicCost = 4.0 * parameters.getCostPerStep() * minSteps;


      double heightCost = 0.0;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int nodeXIndex = node.getXIndex(robotQuadrant);
         int nodeYIndex = node.getYIndex(robotQuadrant);

         PawNodeSnapData nodeData = snapper.snapPawNode(robotQuadrant, nodeXIndex, nodeYIndex, node.getStepYaw());

         if (nodeData == null)
         {
            heightCost = 0.0;
            break;
         }

         RigidBodyTransform nodeTransform = new RigidBodyTransform();

         PawNodeTools.getSnappedNodeTransformToWorld(nodeXIndex, nodeYIndex, nodeData.getSnapTransform(), nodeTransform);

         if (!nodeTransform.containsNaN())
         {
            double heightChange = goalPose.getZ() - nodeTransform.getTranslationVector().getZ();

            if (heightChange > 0.0)
               heightCost += parameters.getStepUpWeight() * heightChange;
            else
               heightCost += -parameters.getStepDownWeight() * heightChange;
         }
      }

      return yawHeuristicCost + stepHeuristicCost + heightCost + parameters.getDistanceWeight() * bodyDistance;
   }
}

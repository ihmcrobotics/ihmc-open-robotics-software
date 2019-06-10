package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.PawNodeCostTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNodeTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PawDistanceAndYawBasedHeuristics extends PawPlanningCostToGoHeuristics
{
   private final PawNodeSnapper snapper;

   public PawDistanceAndYawBasedHeuristics(PawNodeSnapper snapper, PawStepPlannerParametersReadOnly parameters)
   {
      super(parameters);

      this.snapper = snapper;
   }

   @Override
   protected double computeHeuristics(PawNode node, PawNode goalNode)
   {
      double bodyDistance = node.euclideanDistance(goalNode);

      double referenceYaw = PawNodeCostTools.computeReferenceYaw(node.getOrComputeXGaitCenterPoint(), node.getStepYaw(), goalNode, parameters.getFinalTurnProximity());
      double angleDifference = AngleTools.computeAngleDifferenceMinusPiToPi(node.getStepYaw(), referenceYaw);
      double yawHeuristicCost = parameters.getYawWeight() * Math.abs(angleDifference);
      double distanceHeuristicCost = parameters.getDistanceWeight() * bodyDistance;

      double heightCost = 0.0;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int goalNodeXIndex = goalNode.getXIndex(robotQuadrant);
         int goalNodeYIndex = goalNode.getYIndex(robotQuadrant);
         int nodeXIndex = node.getXIndex(robotQuadrant);
         int nodeYIndex = node.getYIndex(robotQuadrant);

         PawNodeSnapData goalNodeData = snapper.snapPawNode(robotQuadrant, goalNodeXIndex, goalNodeYIndex, goalNode.getStepYaw());
         PawNodeSnapData nodeData = snapper.snapPawNode(robotQuadrant, nodeXIndex, nodeYIndex, node.getStepYaw());

         if (nodeData == null || goalNodeData == null)
         {
            heightCost = 0.0;
            break;
         }

         RigidBodyTransform nodeTransform = new RigidBodyTransform();
         RigidBodyTransform goalNodeTransform = new RigidBodyTransform();

         PawNodeTools.getSnappedNodeTransformToWorld(nodeXIndex, nodeYIndex, nodeData.getSnapTransform(), nodeTransform);
         PawNodeTools.getSnappedNodeTransformToWorld(goalNodeXIndex, goalNodeYIndex, goalNodeData.getSnapTransform(), goalNodeTransform);

         if (!nodeTransform.containsNaN() && !goalNodeTransform.containsNaN())
         {
            double heightChange = goalNodeTransform.getTranslationVector().getZ() - nodeTransform.getTranslationVector().getZ();

            if (heightChange > 0.0)
               heightCost += parameters.getStepUpWeight() * heightChange;
            else
               heightCost += Math.abs(parameters.getStepDownWeight() * heightChange);
         }
      }

      return yawHeuristicCost + heightCost + distanceHeuristicCost;
   }
}

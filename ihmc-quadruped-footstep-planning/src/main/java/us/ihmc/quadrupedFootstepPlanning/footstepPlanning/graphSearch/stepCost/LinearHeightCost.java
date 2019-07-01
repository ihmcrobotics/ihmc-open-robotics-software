package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class LinearHeightCost implements FootstepCost
{
   private final FootstepPlannerParameters parameters;
   private final FootstepNodeSnapperReadOnly snapper;

   public LinearHeightCost(FootstepPlannerParameters parameters, FootstepNodeSnapperReadOnly snapper)
   {
      this.parameters = parameters;
      this.snapper = snapper;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {

      RigidBodyTransform startNodeTransform = new RigidBodyTransform();
      RigidBodyTransform endNodeTransform = new RigidBodyTransform();

      RobotQuadrant movingQuadrant = endNode.getMovingQuadrant();

      int endNodeXIndex = endNode.getXIndex(movingQuadrant);
      int endNodeYIndex = endNode.getYIndex(movingQuadrant);
      int startNodeXIndex = startNode.getXIndex(movingQuadrant);
      int startNodeYIndex = startNode.getYIndex(movingQuadrant);

      FootstepNodeSnapData endNodeData = snapper.getSnapData(endNodeXIndex, endNodeYIndex);
      FootstepNodeSnapData startNodeData = snapper.getSnapData(startNodeXIndex, startNodeYIndex);

      if (startNodeData == null || endNodeData == null)
         return 0.0;

      FootstepNodeTools.getSnappedNodeTransformToWorld(startNodeXIndex, startNodeYIndex, startNodeData.getSnapTransform(), startNodeTransform);
      FootstepNodeTools.getSnappedNodeTransformToWorld(endNodeXIndex, endNodeYIndex, endNodeData.getSnapTransform(), endNodeTransform);

      double heightChange = endNodeTransform.getTranslationVector().getZ() - startNodeTransform.getTranslationVector().getZ();

      if (heightChange > 0.0)
         return parameters.getStepUpWeight() * heightChange;
      else
         return -parameters.getStepDownWeight() * heightChange;
   }
}

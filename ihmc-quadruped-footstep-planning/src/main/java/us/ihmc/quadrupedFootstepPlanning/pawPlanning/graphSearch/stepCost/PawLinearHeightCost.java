package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapperReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNodeTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParametersReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PawLinearHeightCost implements PawNodeCost
{
   private final PawPlannerParametersReadOnly parameters;
   private final PawNodeSnapperReadOnly snapper;

   public PawLinearHeightCost(PawPlannerParametersReadOnly parameters, PawNodeSnapperReadOnly snapper)
   {
      this.parameters = parameters;
      this.snapper = snapper;
   }

   @Override
   public double compute(PawNode startNode, PawNode endNode)
   {

      RigidBodyTransform startNodeTransform = new RigidBodyTransform();
      RigidBodyTransform endNodeTransform = new RigidBodyTransform();

      RobotQuadrant movingQuadrant = endNode.getMovingQuadrant();

      int endNodeXIndex = endNode.getXIndex(movingQuadrant);
      int endNodeYIndex = endNode.getYIndex(movingQuadrant);
      int startNodeXIndex = startNode.getXIndex(movingQuadrant);
      int startNodeYIndex = startNode.getYIndex(movingQuadrant);

      PawNodeSnapData endNodeData = snapper.getSnapData(endNodeXIndex, endNodeYIndex);
      PawNodeSnapData startNodeData = snapper.getSnapData(startNodeXIndex, startNodeYIndex);

      if (startNodeData == null || endNodeData == null)
         return 0.0;

      PawNodeTools.getSnappedNodeTransformToWorld(startNodeXIndex, startNodeYIndex, startNodeData.getSnapTransform(), startNodeTransform);
      PawNodeTools.getSnappedNodeTransformToWorld(endNodeXIndex, endNodeYIndex, endNodeData.getSnapTransform(), endNodeTransform);

      double heightChange = endNodeTransform.getTranslationVector().getZ() - startNodeTransform.getTranslationVector().getZ();

      if (heightChange > 0.0)
         return parameters.getStepUpWeight() * heightChange;
      else
         return -parameters.getStepDownWeight() * heightChange;
   }
}

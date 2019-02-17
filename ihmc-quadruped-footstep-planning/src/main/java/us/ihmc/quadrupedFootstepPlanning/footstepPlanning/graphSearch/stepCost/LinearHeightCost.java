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
      double totalCost = 0.0;

      RigidBodyTransform startNodeTransform = new RigidBodyTransform();
      RigidBodyTransform endNodeTransform = new RigidBodyTransform();

      FootstepNodeSnapData endNodeData = snapper.getSnapData(endNode);
      FootstepNodeSnapData startNodeData = snapper.getSnapData(startNode);

      if (startNodeData == null || endNodeData == null)
         return 0.0;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FootstepNodeTools.getSnappedNodeTransformToWorld(robotQuadrant, startNode, startNodeData.getSnapTransform(robotQuadrant), startNodeTransform);
         FootstepNodeTools.getSnappedNodeTransformToWorld(robotQuadrant, endNode, endNodeData.getSnapTransform(robotQuadrant), endNodeTransform);

         double heightChange = endNodeTransform.getTranslationVector().getZ() - startNodeTransform.getTranslationVector().getZ();

         if (heightChange > 0.0)
            totalCost += parameters.getStepUpWeight() * heightChange;
         else
            totalCost += -parameters.getStepDownWeight() * heightChange;
      }

      return totalCost;
   }
}

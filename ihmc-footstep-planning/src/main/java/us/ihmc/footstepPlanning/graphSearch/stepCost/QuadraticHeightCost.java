package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;

public class QuadraticHeightCost implements FootstepCost
{
   private static final double stepHeightScalar = 10.0;

   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepNodeSnapperReadOnly snapper;

   public QuadraticHeightCost(FootstepPlannerParametersReadOnly costParameters, FootstepNodeSnapperReadOnly snapper)
   {
      this.parameters = costParameters;
      this.snapper = snapper;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      FootstepNodeSnapData endNodeData = snapper.getSnapData(endNode);
      FootstepNodeSnapData startNodeData = snapper.getSnapData(startNode);

      if (startNodeData == null || endNodeData == null)
         return 0.0;

      RigidBodyTransform snappedStartNodeTransform = startNodeData.getOrComputeSnappedNodeTransform(startNode);
      RigidBodyTransform snappedEndNodeTransform = startNodeData.getOrComputeSnappedNodeTransform(endNode);

      double heightChange = snappedEndNodeTransform.getTranslationZ() - snappedStartNodeTransform.getTranslationZ();

      if (heightChange > 0.0)
         return parameters.getStepUpWeight() * Math.pow(stepHeightScalar * heightChange, 2.0);
      else
         return parameters.getStepDownWeight() * Math.pow(stepHeightScalar * heightChange, 2.0);
   }
}

package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;

public class PitchAndRollBasedCost implements FootstepCost
{
   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepNodeSnapperReadOnly snapper;

   public PitchAndRollBasedCost(FootstepPlannerParametersReadOnly costParameters, FootstepNodeSnapperReadOnly snapper)
   {
      this.parameters = costParameters;
      this.snapper = snapper;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      FootstepNodeSnapData nodeData = snapper.getSnapData(endNode);

      if (nodeData == null)
         return 0.0;

      double[] yawPitchRoll = new double[3];
      RigidBodyTransformReadOnly nodeTransform = nodeData.getOrComputeSnappedNodeTransform(endNode);
      nodeTransform.getRotationYawPitchRoll(yawPitchRoll);
      double pitch = yawPitchRoll[1];
      double roll = yawPitchRoll[2];

      return parameters.getPitchWeight() * Math.abs(pitch) + parameters.getRollWeight() * Math.abs(roll);
   }
}

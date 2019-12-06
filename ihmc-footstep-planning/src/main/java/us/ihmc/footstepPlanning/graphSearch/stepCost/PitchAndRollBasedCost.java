package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;

import java.util.function.DoubleSupplier;

public class PitchAndRollBasedCost implements FootstepCost
{
   private final FootstepNodeSnapperReadOnly snapper;
   private final DoubleSupplier pitchWeight;
   private final DoubleSupplier rollWeight;

   public PitchAndRollBasedCost(FootstepPlannerParametersReadOnly parameters, FootstepNodeSnapperReadOnly snapper)
   {
      this(parameters::getPitchWeight, parameters::getRollWeight, snapper);
   }

   public PitchAndRollBasedCost(DoubleSupplier pitchWeight, DoubleSupplier rollWeight, FootstepNodeSnapperReadOnly snapper)
   {
      this.pitchWeight = pitchWeight;
      this.rollWeight = rollWeight;
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

      return pitchWeight.getAsDouble() * Math.abs(pitch) + rollWeight.getAsDouble() * Math.abs(roll);
   }
}

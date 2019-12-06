package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.function.DoubleSupplier;

public class EuclideanDistanceHeuristics extends CostToGoHeuristics
{
   public EuclideanDistanceHeuristics(DoubleProvider weight, FootstepPlannerParametersReadOnly parameters, FootstepNodeSnapperReadOnly snapper)
   {
      super(weight::getValue, parameters::getIdealFootstepWidth, snapper);
   }

   @Override
   protected double computeHeuristics(FramePose3DReadOnly pose)
   {
      return pose.getPositionDistance(goalPose);
   }
}

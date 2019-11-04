package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class EuclideanDistanceHeuristics extends CostToGoHeuristics
{
   public EuclideanDistanceHeuristics(DoubleProvider weight, FootstepPlannerParametersReadOnly parameters, FootstepNodeSnapperReadOnly snapper)
   {
      super(weight, parameters, snapper);
   }

   @Override
   protected double computeHeuristics(FramePose3DReadOnly pose)
   {
      return pose.getPositionDistance(goalPose);
   }
}

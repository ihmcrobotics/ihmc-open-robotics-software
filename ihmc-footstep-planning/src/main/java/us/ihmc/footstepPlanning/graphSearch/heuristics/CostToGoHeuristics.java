package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

public abstract class CostToGoHeuristics
{
   private final DoubleProvider weight;
   private final FramePose3D pose = new FramePose3D();
   private final FootstepNodeSnapperReadOnly snapper;

   protected final FramePose3D goalPose = new FramePose3D();

   private final FootstepPlannerParametersReadOnly parameters;

   public CostToGoHeuristics(DoubleProvider weight, FootstepPlannerParametersReadOnly parameters, FootstepNodeSnapperReadOnly snapper)
   {
      this.weight = weight;
      this.parameters = parameters;
      this.snapper = snapper;
   }

   public double getWeight()
   {
      return weight.getValue();
   }

   public double compute(FootstepNode node)
   {
      double height = 0.0;

      FootstepNodeSnapData snapData = snapper.getSnapData(node);
      if (snapData != null)
         height = snapData.getSnapTransform().getTranslationZ();

      Point2DReadOnly midfootPoint = node.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());

      pose.setPosition(midfootPoint.getX(), midfootPoint.getY(), height);
      pose.setOrientationYawPitchRoll(node.getYaw(), 0.0, 0.0);

      return weight.getValue() * computeHeuristics(pose);
   }

   abstract double computeHeuristics(FramePose3DReadOnly pose);

   public void setGoalPose(FramePose3DReadOnly goalPose)
   {
      this.goalPose.set(goalPose);
   }
}

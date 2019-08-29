package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.yoVariables.providers.DoubleProvider;

public abstract class CostToGoHeuristics
{
   private final DoubleProvider weight;
   private final FramePose3D pose = new FramePose3D();
   private final FootstepNodeSnapperReadOnly snapper;

   protected final FramePose3D goalPose = new FramePose3D();

   public CostToGoHeuristics(DoubleProvider weight, FootstepNodeSnapperReadOnly snapper)
   {
      this.weight = weight;
      this.snapper = snapper;
   }

   public double getWeight()
   {
      return weight.getValue();
   }

   public double compute(FootstepNode node)
   {
      pose.setPosition(node.getX(), node.getY(), snapper.getSnapData(node).getSnapTransform().getTranslationZ());
      pose.setOrientationYawPitchRoll(node.getYaw(), 0.0, 0.0);

      return weight.getValue() * computeHeuristics(pose);
   }

   abstract double computeHeuristics(FramePose3D pose);

   public void setGoalPose(FramePose3D goalPose)
   {
      this.goalPose.set(goalPose);
   }
}

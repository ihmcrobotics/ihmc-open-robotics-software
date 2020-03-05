package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.function.DoubleSupplier;

public abstract class CostToGoHeuristics
{
   private final FramePose3D pose = new FramePose3D();
   private final FootstepNodeSnapperReadOnly snapper;

   private final DoubleSupplier weight;
   private final DoubleSupplier idealFootstepWidth;

   protected final FramePose3D goalPose = new FramePose3D();

   public CostToGoHeuristics(DoubleProvider weight, FootstepPlannerParametersReadOnly parameters, FootstepNodeSnapperReadOnly snapper)
   {
      this(weight::getValue, parameters::getIdealFootstepWidth, snapper);
   }

   public CostToGoHeuristics(DoubleSupplier weight, DoubleSupplier idealFootstepWidth, FootstepNodeSnapperReadOnly snapper)
   {
      this.weight = weight;
      this.idealFootstepWidth = idealFootstepWidth;
      this.snapper = snapper;
   }

   public double getWeight()
   {
      return weight.getAsDouble();
   }

   public double compute(FootstepNode node)
   {
      Point3DBasics midfootPoint = new Point3D(node.getOrComputeMidFootPoint(idealFootstepWidth.getAsDouble()));

      FootstepNodeSnapData snapData = snapper.getSnapData(node);
      if (snapData != null && !snapData.getSnapTransform().containsNaN())
         snapData.getSnapTransform().transform(midfootPoint);

      pose.setPosition(midfootPoint);
      pose.setOrientationYawPitchRoll(node.getYaw(), 0.0, 0.0);

      return weight.getAsDouble() * computeHeuristics(pose);
   }

   abstract double computeHeuristics(FramePose3DReadOnly pose);

   public void setGoalPose(FramePose3DReadOnly goalPose)
   {
      this.goalPose.set(goalPose);
   }
}

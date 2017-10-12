package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.stepCost.DistanceAndYawBasedCost;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class BodyPathHeuristics extends CostToGoHeuristics
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final BodyPathPlanner bodyPath;
   private final FootstepPlannerParameters parameters;

   public BodyPathHeuristics(YoVariableRegistry registry, FootstepPlannerParameters parameters, BodyPathPlanner bodyPath)
   {
      super(registry);
      this.bodyPath = bodyPath;
      this.parameters = parameters;
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      Point2D midFootPoint = DistanceAndYawBasedCost.computeMidFootPoint(node, parameters.getIdealFootstepWidth());
      FramePoint3D midFootPoint3D = new FramePoint3D(worldFrame, midFootPoint);

      FramePoint3D closestPointOnPath = new FramePoint3D();
      double alpha = bodyPath.getClosestPoint(midFootPoint3D, closestPointOnPath);
      double distanceToPath = closestPointOnPath.distance(midFootPoint3D);
      double remainingDistance = bodyPath.computePathLength(alpha) + distanceToPath;

      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(node.getYaw(), goalNode.getYaw());
      double minSteps = Math.floor(remainingDistance / parameters.getMaximumStepReach());
      return remainingDistance + parameters.getYawWeight() * Math.abs(yaw) + parameters.getCostPerStep() * minSteps;
   }
}

package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.aStar.CostToGoHeuristics;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DistanceAndYawBasedHeuristics extends CostToGoHeuristics
{
   private final YoDouble yawWeight;

   public DistanceAndYawBasedHeuristics(double yawWeight, YoVariableRegistry registry)
   {
      super(registry);

      String namePrefix = getClass().getSimpleName();
      this.yawWeight = new YoDouble(namePrefix + "YawWeight", registry);
      this.yawWeight.set(yawWeight);
   }

   @Override
   protected double computeHeuristics(FootstepNode node, FootstepNode goalNode)
   {
      Point2D goalPoint = DistanceAndYawBasedCost.computeMidFootPoint(goalNode);
      Point2D nodeMidFootPoint = DistanceAndYawBasedCost.computeMidFootPoint(node);
      double euclideanDistance = nodeMidFootPoint.distance(goalPoint);
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(node.getYaw(), goalNode.getYaw());
      return euclideanDistance + yawWeight.getDoubleValue() * Math.abs(yaw);
   }
}

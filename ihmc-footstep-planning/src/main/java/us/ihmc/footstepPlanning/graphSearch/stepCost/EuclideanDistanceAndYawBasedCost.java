package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.AngleTools;

import java.util.function.DoubleSupplier;

public class EuclideanDistanceAndYawBasedCost implements FootstepCost
{
   private final DoubleSupplier idealFootstepWidth;
   private final DoubleSupplier idealFootstepLength;
   private final DoubleSupplier longStepWeight;
   private final DoubleSupplier forwardWeight;
   private final DoubleSupplier lateralWeight;
   private final DoubleSupplier yawWeight;
   private final DoubleSupplier costPerStep;

   public EuclideanDistanceAndYawBasedCost(FootstepPlannerParametersReadOnly parameters)
   {
      this(parameters::getIdealFootstepWidth,
           parameters::getIdealFootstepLength,
           parameters::getLongStepWeight,
           parameters::getForwardWeight,
           parameters::getLateralWeight,
           parameters::getYawWeight,
           parameters::getCostPerStep);
   }

   public EuclideanDistanceAndYawBasedCost(DoubleSupplier idealFootstepWidth,
                                           DoubleSupplier idealFootstepLength,
                                           DoubleSupplier longStepWeight,
                                           DoubleSupplier forwardWeight,
                                           DoubleSupplier lateralWeight,
                                           DoubleSupplier yawWeight,
                                           DoubleSupplier costPerStep)
   {
      this.idealFootstepWidth = idealFootstepWidth;
      this.idealFootstepLength = idealFootstepLength;
      this.longStepWeight = longStepWeight;
      this.forwardWeight = forwardWeight;
      this.lateralWeight = lateralWeight;
      this.yawWeight = yawWeight;
      this.costPerStep = costPerStep;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      Point2D startPoint = startNode.getOrComputeMidFootPoint(idealFootstepWidth.getAsDouble());
      Point2D endPoint = endNode.getOrComputeMidFootPoint(idealFootstepLength.getAsDouble());
      double euclideanDistance = startPoint.distance(endPoint);
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(startNode.getYaw(), endNode.getYaw());

      double distanceCost;
      if (euclideanDistance > idealFootstepLength.getAsDouble())
         distanceCost = longStepWeight.getAsDouble();
      else
         distanceCost = 0.5 * (forwardWeight.getAsDouble() + lateralWeight.getAsDouble());

      return distanceCost * euclideanDistance + yawWeight.getAsDouble() * Math.abs(yaw) + costPerStep.getAsDouble();
   }
}

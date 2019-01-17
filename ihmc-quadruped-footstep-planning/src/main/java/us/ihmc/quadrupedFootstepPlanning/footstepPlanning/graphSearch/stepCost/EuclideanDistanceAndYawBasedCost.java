package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.commons.MathTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class EuclideanDistanceAndYawBasedCost implements FootstepCost
{
   private final FootstepPlannerParameters parameters;

   public EuclideanDistanceAndYawBasedCost(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(startNode.getNominalYaw(), endNode.getNominalYaw());
      double totalCost = parameters.getYawWeight() * Math.abs(yaw) + parameters.getCostPerStep();
      double distanceWeight = 0.5 * (parameters.getForwardWeight() + parameters.getLateralWeight());

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         double euclideanDistance = Math.sqrt(MathTools.square(startNode.getX(robotQuadrant) - endNode.getX(robotQuadrant)) +
                                                    MathTools.square(startNode.getY(robotQuadrant) - endNode.getY(robotQuadrant)));

         totalCost += distanceWeight * euclideanDistance;
      }

      return totalCost;
   }
}

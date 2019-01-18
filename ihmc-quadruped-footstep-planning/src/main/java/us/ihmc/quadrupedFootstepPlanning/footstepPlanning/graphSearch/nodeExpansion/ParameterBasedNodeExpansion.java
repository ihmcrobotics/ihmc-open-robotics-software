package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;
import java.util.HashSet;

import static us.ihmc.robotics.robotSide.RobotQuadrant.*;

public class ParameterBasedNodeExpansion implements FootstepNodeExpansion
{
   private FootstepNode goalNode;
   private final FootstepPlannerParameters parameters;

   public ParameterBasedNodeExpansion(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   public void setGoalNode(FootstepNode goalNode)
   {
      this.goalNode = goalNode;
   }

   @Override
   public HashSet<FootstepNode> expandNode(FootstepNode node)
   {
      HashSet<FootstepNode> expansion = new HashSet<>();
      addDefaultFootsteps(node, expansion);

      return expansion;
   }

   private void addGoalNodeIfReachable(FootstepNode node, HashSet<FootstepNode> expansion)
   {
      if (node.euclideanDistance(goalNode) < parameters.getMaximumStepCycleDistance())
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (node.quadrantEuclideanDistance(robotQuadrant, goalNode) >= parameters.getMaximumStepReach())
               return;
         }

         expansion.add(goalNode);
      }

   }

   private void addDefaultFootsteps(FootstepNode node, HashSet<FootstepNode> neighboringNodesToPack)
   {
      for (double movingX = parameters.getMinimumStepLength(); movingX < parameters.getMaximumStepReach(); movingX += FootstepNode.gridSizeXY)
      {
         for (double movingY = parameters.getMinimumStepWidth(); movingY < parameters.getMaximumStepWidth(); movingY += FootstepNode.gridSizeXY)
         {
            FootstepNode offsetNode = constructNodeInPreviousNodeFrame(movingX, movingY, node);
            neighboringNodesToPack.add(offsetNode);
         }
      }
   }

   private static FootstepNode constructNodeInPreviousNodeFrame(double movingX, double movingY, FootstepNode previousNode)
   {
      Vector2D movingVector = new Vector2D(movingX, movingY);

      double nodeYaw = previousNode.getNominalYaw();

      AxisAngle rotation = new AxisAngle(nodeYaw, 0.0, 0.0);
      rotation.transform(movingVector);

      RobotQuadrant nextQuadrant = previousNode.getMovingQuadrant().getNextRegularGaitSwingQuadrant();
      Point2D frontLeft = new Point2D(previousNode.getX(FRONT_LEFT), previousNode.getY(FRONT_LEFT));
      Point2D frontRight = new Point2D(previousNode.getX(FRONT_RIGHT), previousNode.getY(FRONT_RIGHT));
      Point2D hindLeft = new Point2D(previousNode.getX(HIND_LEFT), previousNode.getY(HIND_LEFT));
      Point2D hindRight = new Point2D(previousNode.getX(HIND_RIGHT), previousNode.getY(HIND_RIGHT));

      switch (nextQuadrant)
      {
      case FRONT_LEFT:
         frontLeft.add(movingX, movingY);
         break;
      case FRONT_RIGHT:
         frontRight.add(movingX, movingY);
         break;
      case HIND_LEFT:
         hindLeft.add(movingX, movingY);
         break;
      default:
         hindRight.add(movingX, movingY);
         break;
      }

      return new FootstepNode(nextQuadrant, frontLeft, frontRight, hindLeft, hindRight);
   }

   private static ArrayList<Double> constructArrayFromEndpointsAndSpacing(double minValue, double maxValue, double spacing)
   {
      if (maxValue < minValue)
         throw new RuntimeException("Max value: " + maxValue + " should be less than min value: " + minValue);

      ArrayList<Double> array = new ArrayList<>();
      double value = minValue;

      while (value < maxValue)
      {
         array.add(value);
         value += spacing;
      }

      return array;
   }
}

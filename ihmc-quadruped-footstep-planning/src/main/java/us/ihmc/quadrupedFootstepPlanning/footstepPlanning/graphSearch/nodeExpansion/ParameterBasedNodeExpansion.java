package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.euclid.axisAngle.AxisAngle;
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
      for (double frontLeftX = parameters.getMinimumStepLength(); frontLeftX < parameters.getMaximumStepReach(); frontLeftX += FootstepNode.gridSizeXY)
      {
         for (double frontLeftY = parameters.getMinimumStepWidth(); frontLeftY < parameters.getMaximumStepWidth(); frontLeftY += FootstepNode.gridSizeXY)
         {
            for (double frontRightX = parameters.getMinimumStepLength(); frontRightX < parameters.getMaximumStepReach(); frontRightX += FootstepNode.gridSizeXY)
            {
               for (double frontRightY = parameters.getMinimumStepWidth(); frontRightY < parameters.getMaximumStepWidth(); frontRightY += FootstepNode.gridSizeXY)
               {
                  for (double hindLeftX = parameters.getMinimumStepLength(); hindLeftX < parameters.getMaximumStepReach(); hindLeftX += FootstepNode.gridSizeXY)
                  {
                     for (double hindLeftY = parameters.getMinimumStepWidth(); hindLeftY < parameters.getMaximumStepWidth(); hindLeftY += FootstepNode.gridSizeXY)
                     {
                        for (double hindRightX = parameters.getMinimumStepLength(); hindRightX < parameters.getMaximumStepReach(); hindRightX += FootstepNode.gridSizeXY)
                        {
                           for (double hindRightY = parameters.getMinimumStepWidth(); hindRightY < parameters.getMaximumStepWidth(); hindRightY += FootstepNode.gridSizeXY)
                           {
                              FootstepNode offsetNode = constructNodeInPreviousNodeFrame(frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY,
                                                                                         hindRightX, hindRightY, node);
                              neighboringNodesToPack.add(offsetNode);
                           }
                        }
                     }
                  }
               }
            }
         }
      }
   }

   private static FootstepNode constructNodeInPreviousNodeFrame(double frontLeftX, double frontLeftY, double frontRightX, double frontRightY, double hindLeftX,
                                                                double hindLeftY, double hindRightX, double hindRightY, FootstepNode node)
   {
      Vector2D frontLeft = new Vector2D(frontLeftX, frontLeftY);
      Vector2D frontRight = new Vector2D(frontRightX, frontRightY);
      Vector2D hindLeft = new Vector2D(hindLeftX, hindLeftY);
      Vector2D hindRight = new Vector2D(hindRightX, hindRightY);

      double nodeYaw = node.getNominalYaw();

      AxisAngle rotation = new AxisAngle(nodeYaw, 0.0, 0.0);
      rotation.transform(frontLeft);
      rotation.transform(frontRight);
      rotation.transform(hindLeft);
      rotation.transform(hindRight);

      RobotQuadrant nextQuadrant = node.getMovingQuadrant().getNextRegularGaitSwingQuadrant();

      return new FootstepNode(nextQuadrant, frontLeft, frontRight, hindLeft, hindRight);
   }


   private static ArrayList<Double> constructArrayFromEndpointsAndSpacing(double minValue, double maxValue, double spacing)
   {
      if(maxValue < minValue)
         throw new RuntimeException("Max value: " + maxValue + " should be less than min value: " + minValue);

      ArrayList<Double> array = new ArrayList<>();
      double value = minValue;

      while(value < maxValue)
      {
         array.add(value);
         value += spacing;
      }

      return array;
   }
}

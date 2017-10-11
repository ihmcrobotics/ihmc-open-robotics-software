package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import java.util.ArrayList;
import java.util.HashSet;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ParameterBasedNodeExpansion implements FootstepNodeExpansion
{
   private SideDependentList<FootstepNode> goalNodes;
   private final FootstepPlannerParameters parameters;

   private final ArrayList<Double> xOffsetFromIdealFootstep;
   private final ArrayList<Double> yOffsetFromIdealFootstep;
   private final ArrayList<Double> yawOffsetFromIdealFootstep;

   public ParameterBasedNodeExpansion(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;

      xOffsetFromIdealFootstep = constructArrayFromEndpointsAndSpacing(parameters.getMinimumStepLength(), parameters.getMaximumStepReach(), FootstepNode.gridSizeXY);
      yOffsetFromIdealFootstep = constructArrayFromEndpointsAndSpacing(parameters.getMinimumStepWidth(), parameters.getMaximumStepWidth(), FootstepNode.gridSizeXY);
      yawOffsetFromIdealFootstep = constructArrayFromEndpointsAndSpacing(- parameters.getMaximumStepYaw(), parameters.getMaximumStepYaw(), FootstepNode.gridSizeYaw);
   }

   public void setGoalNodes(SideDependentList<FootstepNode> goalNodes)
   {
      this.goalNodes = goalNodes;
   }

   @Override
   public HashSet<FootstepNode> expandNode(FootstepNode node)
   {
      HashSet<FootstepNode> expansion = new HashSet<>();

      addGoalNodeIfReachable(node, expansion);
      addDefaultFootsteps(node, expansion);

      return expansion;
   }

   private void addGoalNodeIfReachable(FootstepNode node, HashSet<FootstepNode> expansion)
   {
      RobotSide nextSide = node.getRobotSide().getOppositeSide();
      FootstepNode goalNode = goalNodes.get(nextSide);
      double distanceToGoal = node.euclideanDistance(goalNode);
      if(distanceToGoal < parameters.getMaximumStepReach())
      {
         expansion.add(goalNode);
      }
   }

   private void addDefaultFootsteps(FootstepNode node, HashSet<FootstepNode> expansion)
   {
      RobotSide nextSide = node.getRobotSide().getOppositeSide();
      for(double x : xOffsetFromIdealFootstep)
      {
         for(double y : yOffsetFromIdealFootstep)
         {
            for (double yaw : yawOffsetFromIdealFootstep)
            {
               FootstepNode offsetNode = constructNodeInPreviousNodeFrame(x, nextSide.negateIfRightSide(y), nextSide.negateIfRightSide(yaw), node);
               expansion.add(offsetNode);
            }
         }
      }
   }

   private static FootstepNode constructNodeInPreviousNodeFrame(double stepLength, double stepWidth, double stepYaw, FootstepNode node)
   {
      Vector2D footstep = new Vector2D(stepLength, stepWidth);
      AxisAngle rotation = new AxisAngle(node.getYaw(), 0.0, 0.0);
      rotation.transform(footstep);

      return new FootstepNode(node.getX() + footstep.getX(), node.getY() + footstep.getY(), stepYaw + node.getYaw(), node.getRobotSide().getOppositeSide());
   }

   private static FootstepNode constructNodeOffsetFromAnotherNode(double xOffset, double yOffset, double yawOffset, FootstepNode node)
   {
      Vector2D offset = new Vector2D(xOffset, yOffset);
      double yaw = node.getYaw() + yawOffset;
      AxisAngle rotation = new AxisAngle(node.getYaw(), 0.0, 0.0);
      rotation.transform(offset);
      return new FootstepNode(node.getX() + offset.getX(), node.getY() + offset.getY(), yaw, node.getRobotSide());
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

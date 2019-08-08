package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import java.util.ArrayList;
import java.util.HashSet;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ParameterBasedNodeExpansion implements FootstepNodeExpansion
{
   private SideDependentList<FootstepNode> goalNodes;
   private final FootstepPlannerParametersReadOnly parameters;

   public ParameterBasedNodeExpansion(FootstepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   public void setGoalNodes(SideDependentList<FootstepNode> goalNodes)
   {
      this.goalNodes = goalNodes;
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
      double reachSquared = MathTools.square(parameters.getMaximumStepReach());
      for (double x = parameters.getMinimumStepLength(); x <= parameters.getMaximumStepReach(); x += LatticeNode.gridSizeXY)
      {
         for (double y = parameters.getMinimumStepWidth(); y <= parameters.getMaximumStepWidth(); y += LatticeNode.gridSizeXY)
         {
            if (MathTools.square(x) + MathTools.square(y) > reachSquared)
               continue;

            if (Math.abs(x) <= parameters.getMinXClearanceFromStance() && Math.abs(y) <= parameters.getMinYClearanceFromStance())
            {
               continue;
            }

            for (double yaw = parameters.getMinimumStepYaw(); yaw <= parameters.getMaximumStepYaw(); yaw += LatticeNode.gridSizeYaw)
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

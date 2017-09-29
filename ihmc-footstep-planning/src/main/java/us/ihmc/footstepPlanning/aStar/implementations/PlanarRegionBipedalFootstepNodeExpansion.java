package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.AxisAngleTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.footstepPlanning.FootstepPlannerUtils;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.footstepPlanning.aStar.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerParameters;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Optional;

public class PlanarRegionBipedalFootstepNodeExpansion implements FootstepNodeExpansion
{
   private SideDependentList<FootstepNode> goalNodes;
   private final BipedalFootstepPlannerParameters parameters;

   private final double xOffsetFromIdealFootstep[];
   private final double yOffsetFromIdealFootstep[];
   private final double yawOffsetFromIdealFootstep[];

   public PlanarRegionBipedalFootstepNodeExpansion(BipedalFootstepPlannerParameters parameters)
   {
      this.parameters = parameters;

      this.xOffsetFromIdealFootstep = new double[] {parameters.getMinimumStepLength() + FootstepNode.gridSizeX, 0.0,
            parameters.getIdealFootstepLength() - FootstepNode.gridSizeX, parameters.getIdealFootstepLength()};
      this.yOffsetFromIdealFootstep = new double[] {parameters.getMinimumStepWidth() + FootstepNode.gridSizeY,
            parameters.getIdealFootstepWidth() - FootstepNode.gridSizeY, parameters.getIdealFootstepWidth(),
            parameters.getMaximumStepWidth() - FootstepNode.gridSizeY};
      this.yawOffsetFromIdealFootstep = new double[] {- FootstepNode.gridSizeYaw, 0.0, FootstepNode.gridSizeYaw, parameters.getMaximumStepYaw() - FootstepNode.gridSizeYaw};
   }

   public void setGoalNodes(SideDependentList<FootstepNode> goalNodes)
   {
      this.goalNodes = goalNodes;
   }

   @Override
   public HashSet<FootstepNode> expandNode(FootstepNode node)
   {
      HashSet<FootstepNode> expansion = new HashSet<>();

      Optional<FootstepNode> goalNode = addGoalNodeIfReachable(node, expansion);
      if(goalNode.isPresent())
      {
         addOffsetsFromIdealFootstep(goalNode.get(), expansion);
      }
      else
      {
         FootstepNode idealFootstepNode = computeAndAddIdealFootstep(node, expansion);
         addOffsetsFromIdealFootstep(idealFootstepNode, expansion);
      }

      addSideStep(node, expansion);

      return expansion;
   }

   private Optional<FootstepNode> addGoalNodeIfReachable(FootstepNode node, HashSet<FootstepNode> expansion)
   {
      RobotSide nextSide = node.getRobotSide().getOppositeSide();
      FootstepNode goalNode = goalNodes.get(nextSide);
      double distanceToGoal = node.euclideanDistance(goalNode);
      if(distanceToGoal < parameters.getMaximumStepReach())
      {
         expansion.add(goalNode);
         return Optional.of(goalNode);
      }
      return Optional.empty();
   }

   private FootstepNode computeAndAddIdealFootstep(FootstepNode node, HashSet<FootstepNode> expansion)
   {
      RobotSide nextSide = node.getRobotSide().getOppositeSide();
      double idealFootstepWidth = nextSide.negateIfRightSide(parameters.getIdealFootstepWidth());
      FootstepNode idealFootstep = constructNodeInPreviousNodeFrame(parameters.getIdealFootstepLength(), idealFootstepWidth, 0.0, node);
      expansion.add(idealFootstep);
      return idealFootstep;
   }

   private void addSideStep(FootstepNode node, HashSet<FootstepNode> expansion)
   {
      RobotSide nextSide = node.getRobotSide().getOppositeSide();
      double stepWidth = nextSide.negateIfRightSide(parameters.getIdealFootstepWidth());
      FootstepNode sideStep = constructNodeInPreviousNodeFrame(0.0, stepWidth, 0.0, node);
      expansion.add(sideStep);
   }

   private void addOffsetsFromIdealFootstep(FootstepNode node, HashSet<FootstepNode> expansion)
   {
      RobotSide nextSide = node.getRobotSide().getOppositeSide();
      for(double x : xOffsetFromIdealFootstep)
      {
         for(double y : yOffsetFromIdealFootstep)
         {
            for (double yaw : yawOffsetFromIdealFootstep)
            {
               if (x == 0 && y == 0 && yaw == 0)
                  continue;
               FootstepNode offsetNode = constructNodeOffsetFromAnotherNode(x, nextSide.negateIfRightSide(y), nextSide.negateIfRightSide(yaw), node);
               expansion.add(offsetNode);
            }
         }
      }
   }

   private static FootstepNode constructNodeInPreviousNodeFrame(double stepLength, double stepWidth, double stepYaw, FootstepNode node)
   {
      Vector2D footstep = new Vector2D(stepLength, stepWidth);
      AxisAngle rotation = new AxisAngle(0.0, 0.0, 1.0, node.getYaw());
      rotation.transform(footstep);

      return new FootstepNode(node.getX() + footstep.getX(), node.getY() + footstep.getY(), stepYaw + node.getYaw(), node.getRobotSide().getOppositeSide());
   }

   private static FootstepNode constructNodeOffsetFromAnotherNode(double xOffset, double yOffset, double yawOffset, FootstepNode node)
   {
      Vector2D offset = new Vector2D(xOffset, yOffset);
      double yaw = node.getYaw() + yawOffset;
      AxisAngle rotation = new AxisAngle(yaw, 0.0, 0.0);
      rotation.transform(offset);
      return new FootstepNode(node.getX() + offset.getX(), node.getY() + offset.getY(), yaw, node.getRobotSide());
   }
}

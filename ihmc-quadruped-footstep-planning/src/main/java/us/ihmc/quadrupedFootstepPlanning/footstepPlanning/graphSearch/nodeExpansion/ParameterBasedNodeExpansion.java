package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.HashSet;

import static us.ihmc.robotics.robotSide.RobotQuadrant.*;

public class ParameterBasedNodeExpansion implements FootstepNodeExpansion
{
   private FootstepNode goalNode;
   private final FootstepPlannerParameters parameters;
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;

   public ParameterBasedNodeExpansion(FootstepPlannerParameters parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.parameters = parameters;
      this.xGaitSettings = xGaitSettings;
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
      Orientation3DReadOnly nodeOrientation = new AxisAngle(node.getYaw(), 0.0, 0.0);

      Vector2D clearanceVector = new Vector2D(parameters.getMinXClearanceFromFoot(), parameters.getMinYClearanceFromFoot());
      nodeOrientation.transform(clearanceVector);


      for (double movingX = parameters.getMinimumStepLength(); movingX < parameters.getMaximumStepReach(); movingX += FootstepNode.gridSizeXY)
      {
         for (double movingY = parameters.getMinimumStepWidth(); movingY < parameters.getMaximumStepWidth(); movingY += FootstepNode.gridSizeXY)
         {
            Vector2D movingVector = new Vector2D(movingX, movingY);
            nodeOrientation.transform(movingVector);

            Point2D newXGaitPosition = new Point2D(node.getOrComputeXGaitCenterPoint());
            newXGaitPosition.add(movingVector);

            for (double yawChange = parameters.getMinimumStepYaw(); yawChange < parameters.getMaximumStepYaw(); yawChange += FootstepNode.gridSizeYaw)
            {
               Vector2D footOffset = new Vector2D(xGaitSettings.getStanceLength(), xGaitSettings.getStanceWidth());
               footOffset.scale(0.5);

               double newYaw = node.getYaw() + yawChange;
               Orientation3DReadOnly newOrientation =  new AxisAngle(newYaw, 0.0, 0.0);
               newOrientation.transform(footOffset);

               Point2D newNodePosition = new Point2D(newXGaitPosition);
               newNodePosition.add(footOffset);

               if (!checkNodeIsFarEnoughFromOtherFeet(newNodePosition, clearanceVector, node))
                  continue;
               if (MathTools.epsilonEquals(movingVector.lengthSquared(), 0.0, 1e-3))
                  continue;

               FootstepNode offsetNode = constructNodeInPreviousNodeFrame(newNodePosition, newYaw, node, xGaitSettings);

               if (offsetNode.geometricallyEquals(node))
                  throw new RuntimeException("This shouldn't be created.");

               neighboringNodesToPack.add(offsetNode);
            }
         }
      }
   }



   static boolean checkNodeIsFarEnoughFromOtherFeet(Point2DReadOnly nodePositionToCheck, Vector2DReadOnly requiredClearance, FootstepNode previousNode)
   {
      RobotQuadrant nextQuadrant = previousNode.getMovingQuadrant().getNextRegularGaitSwingQuadrant();

      for (RobotQuadrant otherQuadrant : RobotQuadrant.values)
      {
         if (nextQuadrant == otherQuadrant)
            continue;

         double otherNodeX = previousNode.getX(otherQuadrant);
         double otherNodeY = previousNode.getY(otherQuadrant);

         if (!checkNodeIsFarEnoughFromOtherFoot(nodePositionToCheck, requiredClearance, otherNodeX, otherNodeY))
            return false;
      }

      return true;
   }

   /**
    * Checks if the node position being added {@param nodePositionToCheck} is too close to another foot position, defined by ({@param otherFootX}, {@param otherFootY})
    *
    */
   static boolean checkNodeIsFarEnoughFromOtherFoot(Point2DReadOnly nodePositionToCheck, Vector2DReadOnly requiredClearance, double otherFootX, double otherFootY)
   {
      double maxForward = otherFootX + Math.abs(requiredClearance.getX());
      double maxBackward = otherFootX - Math.abs(requiredClearance.getX());

      double maxLeft = otherFootY + Math.abs(requiredClearance.getY());
      double maxRight = otherFootY - Math.abs(requiredClearance.getY());

      if (MathTools.intervalContains(nodePositionToCheck.getX(), maxBackward, maxForward, true, true) && MathTools.intervalContains(nodePositionToCheck.getY(),
                                                                                                                                    maxRight, maxLeft, true, true))
         return false;
      else
         return true;
   }

   private static FootstepNode constructNodeInPreviousNodeFrame(Point2DReadOnly newNodePosition, double newNodeYaw, FootstepNode previousNode,
                                                                QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      RobotQuadrant nextQuadrant = previousNode.getMovingQuadrant().getNextRegularGaitSwingQuadrant();
      Point2D frontLeft = new Point2D(previousNode.getX(FRONT_LEFT), previousNode.getY(FRONT_LEFT));
      Point2D frontRight = new Point2D(previousNode.getX(FRONT_RIGHT), previousNode.getY(FRONT_RIGHT));
      Point2D hindLeft = new Point2D(previousNode.getX(HIND_LEFT), previousNode.getY(HIND_LEFT));
      Point2D hindRight = new Point2D(previousNode.getX(HIND_RIGHT), previousNode.getY(HIND_RIGHT));

      switch (nextQuadrant)
      {
      case FRONT_LEFT:
         frontLeft.set(newNodePosition);
         break;
      case FRONT_RIGHT:
         frontRight.set(newNodePosition);
         break;
      case HIND_LEFT:
         hindLeft.set(newNodePosition);
         break;
      default:
         hindRight.set(newNodePosition);
         break;
      }

      return new FootstepNode(nextQuadrant, frontLeft, frontRight, hindLeft, hindRight, newNodeYaw, xGaitSettings.getStanceLength(),
                              xGaitSettings.getStanceWidth());
   }
}

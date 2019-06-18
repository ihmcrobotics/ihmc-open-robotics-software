package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
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
   protected final FootstepPlannerParameters parameters;
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;

   public ParameterBasedNodeExpansion(FootstepPlannerParameters parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.parameters = parameters;
      this.xGaitSettings = xGaitSettings;
   }

   @Override
   public HashSet<FootstepNode> expandNode(FootstepNode node)
   {
      HashSet<FootstepNode> expansion = new HashSet<>();
      addDefaultFootsteps(node, expansion, FootstepNode.gridSizeXY);

      return expansion;
   }

   protected void addDefaultFootsteps(FootstepNode node, HashSet<FootstepNode> neighboringNodesToPack, double resolution)
   {
      RobotQuadrant movingQuadrant = node.getMovingQuadrant();
      RobotQuadrant nextQuadrant;
      if (xGaitSettings.getEndPhaseShift() > 180.0)
         nextQuadrant = movingQuadrant.getNextReversedRegularGaitSwingQuadrant();
      else
         nextQuadrant = movingQuadrant.getNextRegularGaitSwingQuadrant();

      int oldXIndex = node.getXIndex(nextQuadrant);
      int oldYIndex = node.getYIndex(nextQuadrant);

      Point2DReadOnly xGaitCenterPoint = node.getOrComputeXGaitCenterPoint();
//      double previousYaw = node.getYaw();
      double previousYaw = node.getNominalYaw();
      Orientation3DReadOnly nodeOrientation = new AxisAngle(previousYaw, 0.0, 0.0);

      Vector2D clearanceVector = new Vector2D(parameters.getMinXClearanceFromFoot(), parameters.getMinYClearanceFromFoot());
      nodeOrientation.transform(clearanceVector);

      boolean isMovingFront = movingQuadrant.isQuadrantInFront();
      boolean isMovingLeft = movingQuadrant.isQuadrantOnLeftSide();
      double maxReach = isMovingFront ? parameters.getMaximumFrontStepReach() : parameters.getMaximumHindStepReach();
      double minLength = isMovingFront ? parameters.getMinimumFrontStepLength() : parameters.getMinimumHindStepLength();
      double maxLength = isMovingFront ? parameters.getMaximumFrontStepLength() : parameters.getMaximumHindStepLength();
      double maxWidth = isMovingLeft ? parameters.getMaximumStepWidth() : -parameters.getMinimumStepWidth();
      double minWidth = isMovingLeft ? parameters.getMinimumStepWidth() : -parameters.getMaximumStepWidth();

      for (double movingX = minLength; movingX < maxLength; movingX += resolution)
      {
         for (double movingY = minWidth; movingY < maxWidth; movingY += resolution)
         {
            if (EuclidCoreTools.norm(movingX, movingY) > maxReach)
               continue;

            Vector2D movingVector = new Vector2D(movingX, movingY);
            nodeOrientation.transform(movingVector);

            Point2D newXGaitPosition = new Point2D(xGaitCenterPoint);
            newXGaitPosition.add(movingVector);

            Vector2D footOffset = new Vector2D(nextQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength()),
                                               nextQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth()));
            footOffset.scale(0.5);

            nodeOrientation.transform(footOffset);

            Point2D newNodePosition = new Point2D(newXGaitPosition);
            newNodePosition.add(footOffset);

            int xIndex = FootstepNode.snapToGrid(newNodePosition.getX());
            int yIndex = FootstepNode.snapToGrid(newNodePosition.getY());

            if (!checkNodeIsFarEnoughFromOtherFeet(newNodePosition, clearanceVector, node))
               continue;
            if (xIndex == oldXIndex && yIndex == oldYIndex)
               continue;

            FootstepNode offsetNode = constructNodeInPreviousNodeFrame(newNodePosition, node, xGaitSettings);

            neighboringNodesToPack.add(offsetNode);
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

   private static FootstepNode constructNodeInPreviousNodeFrame(Point2DReadOnly newNodePosition, FootstepNode previousNode,
                                                                QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      RobotQuadrant nextQuadrant = previousNode.getMovingQuadrant().getNextRegularGaitSwingQuadrant();
      return FootstepNode.constructNodeFromOtherNode(nextQuadrant, newNodePosition, previousNode, xGaitSettings.getStanceLength(),
                                                     xGaitSettings.getStanceWidth());
   }
}

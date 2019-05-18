package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.CliffDetectionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.HashSet;

import static us.ihmc.robotics.robotSide.RobotQuadrant.*;

public class VariableResolutionNodeExpansion implements FootstepNodeExpansion
{
   private final FootstepPlannerParameters parameters;
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final FootstepNodeSnapperReadOnly snapper;

   public VariableResolutionNodeExpansion(FootstepPlannerParameters parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings, FootstepNodeSnapperReadOnly snapper)
   {
      this.parameters = parameters;
      this.xGaitSettings = xGaitSettings;
      this.snapper = snapper;
   }

   @Override
   public HashSet<FootstepNode> expandNode(FootstepNode node)
   {
      HashSet<FootstepNode> expansion = new HashSet<>();
      double resolution = getExpansionResolution(node);
      addDefaultFootsteps(node, expansion, resolution);

      return expansion;
   }

   private double getExpansionResolution(FootstepNode node)
   {
      if (!snapper.hasPlanarRegions())
         return 2 * FootstepNode.gridSizeXY;

      RobotQuadrant movingQuadrant = node.getMovingQuadrant();
      int xIndex = node.getXIndex(movingQuadrant);
      int yIndex = node.getYIndex(movingQuadrant);
      RigidBodyTransform footTransformToWorld = new RigidBodyTransform();
      FootstepNodeTools.getSnappedNodeTransformToWorld(xIndex, yIndex, snapper.getSnapData(xIndex, yIndex).getSnapTransform(), footTransformToWorld);

      Point3D footInWorld = new Point3D();
      footTransformToWorld.transform(footInWorld);

      if (CliffDetectionTools.isNearCliff(movingQuadrant, snapper.getPlanarRegionsList(), footInWorld, node.getNominalYaw(), parameters))
         return FootstepNode.gridSizeXY;

      return 2 * FootstepNode.gridSizeXY;
   }


   private void addDefaultFootsteps(FootstepNode node, HashSet<FootstepNode> neighboringNodesToPack, double expansionResolution)
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

      double maxReach = movingQuadrant.isQuadrantInFront() ? parameters.getMaximumFrontStepReach() : parameters.getMaximumHindStepReach();
      double minLength = movingQuadrant.isQuadrantInFront() ? parameters.getMinimumFrontStepLength() : parameters.getMinimumHindStepLength();
      double maxLength = movingQuadrant.isQuadrantInFront() ? parameters.getMaximumFrontStepLength() : parameters.getMaximumHindStepLength();

      for (double movingX = minLength; movingX < maxLength; movingX += expansionResolution)
      {
         for (double movingY = parameters.getMinimumStepWidth(); movingY < parameters.getMaximumStepWidth(); movingY += expansionResolution)
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

      return new FootstepNode(nextQuadrant, frontLeft, frontRight, hindLeft, hindRight, xGaitSettings.getStanceLength(),
                              xGaitSettings.getStanceWidth());
   }
}

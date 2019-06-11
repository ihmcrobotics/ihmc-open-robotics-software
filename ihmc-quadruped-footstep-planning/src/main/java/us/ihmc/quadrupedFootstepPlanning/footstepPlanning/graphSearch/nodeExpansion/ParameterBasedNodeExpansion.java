package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.HashSet;

import static us.ihmc.robotics.robotSide.RobotQuadrant.*;

public class ParameterBasedNodeExpansion implements FootstepNodeExpansion
{
   protected final FootstepPlannerParameters parameters;
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;

   private static final double maxDeviationFromXGait = 0.1;

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

   protected void addDefaultFootsteps(FootstepNode nodeToExpand, HashSet<FootstepNode> neighboringNodesToPack, double resolution)
   {
      RobotQuadrant movingQuadrant = nodeToExpand.getMovingQuadrant();
      RobotQuadrant nextQuadrant;
      if (xGaitSettings.getEndPhaseShift() > 180.0)
         nextQuadrant = movingQuadrant.getNextReversedRegularGaitSwingQuadrant();
      else
         nextQuadrant = movingQuadrant.getNextRegularGaitSwingQuadrant();

      double durationBetweenSteps = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(movingQuadrant, xGaitSettings);

      Point2DReadOnly previousXGaitCenterPoint = nodeToExpand.getOrComputeXGaitCenterPoint();
      double previousYaw = nodeToExpand.getStepYaw();
      Orientation3DReadOnly previousNodeOrientation = new AxisAngle(previousYaw, 0.0, 0.0);

      Vector2D clearanceVector = new Vector2D(parameters.getMinXClearanceFromFoot(), parameters.getMinYClearanceFromFoot());
      previousNodeOrientation.transform(clearanceVector);

      boolean isMovingFront = movingQuadrant.isQuadrantInFront();
      double maxReach = isMovingFront ? parameters.getMaximumFrontStepReach() : parameters.getMaximumHindStepReach();

      double maxForwardSpeed = xGaitSettings.getMaxSpeed();
      double maxForwardDisplacement = durationBetweenSteps * maxForwardSpeed;
      double maxLateralDisplacement = xGaitSettings.getMaxHorizontalSpeedFraction() * maxForwardDisplacement;
      double maxYawDisplacement = xGaitSettings.getMaxYawSpeedFraction() * maxForwardDisplacement;

      double maxLength = maxForwardDisplacement + maxDeviationFromXGait;
      double maxWidth = maxLateralDisplacement + maxDeviationFromXGait;

      Vector2D nominalFootOffset = new Vector2D(0.5 * nextQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength()),
                                                0.5 * nextQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth()));
      Point2D newXGaitPosition = new Point2D();

      // FIXME revisit this and see if the operations can be reduced.
      for (double movingX = -maxLength; movingX <= maxLength; movingX += resolution)
      {
         for (double movingY = -maxWidth; movingY <= maxWidth; movingY += resolution)
         {
            double translation = EuclidCoreTools.normSquared(movingX, movingY);

            if (translation > maxReach)
               continue;

            double ellipticalVelocity = MathTools.square(movingX / maxLength) + MathTools.square(movingY / maxWidth);
            if (ellipticalVelocity > 1.0)
               continue;

            double absoluteMaxYawDisplacement = InterpolationTools.hermiteInterpolate(maxYawDisplacement, 0.0, translation / maxReach);
            double minYaw = Math.min(parameters.getMinimumStepYaw(), -absoluteMaxYawDisplacement);
            double maxYaw = Math.min(parameters.getMaximumStepYaw(), absoluteMaxYawDisplacement);

            Vector2D movingVector = new Vector2D(movingX, movingY);
            previousNodeOrientation.transform(movingVector);

            newXGaitPosition.add(previousXGaitCenterPoint, movingVector);

            for (double yaw = minYaw; yaw <= maxYaw; yaw += FootstepNode.gridSizeYaw)
            {
               double newYaw = AngleTools.trimAngleMinusPiToPi(previousYaw + yaw);
               Orientation3DReadOnly newNodeOrientation = new Quaternion(newYaw, 0.0, 0.0);

               Vector2D footOffset = new Vector2D(nominalFootOffset);
               newNodeOrientation.transform(footOffset);

               Point2D newNodePosition = new Point2D(newXGaitPosition);
               newNodePosition.add(footOffset);

               if (!checkNodeIsFarEnoughFromOtherFeet(newNodePosition, clearanceVector, nodeToExpand))
                  continue;

               FootstepNode offsetNode = constructNodeInPreviousNodeFrame(newNodePosition, newYaw, nodeToExpand, xGaitSettings);

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
      return FootstepNode.constructNodeFromOtherNode(nextQuadrant, newNodePosition, newNodeYaw, previousNode, xGaitSettings.getStanceLength(),
                                                     xGaitSettings.getStanceWidth());
   }
}

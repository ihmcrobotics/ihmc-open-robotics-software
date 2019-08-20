package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeExpansion;

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
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParametersReadOnly;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.HashSet;

public class ParameterBasedPawNodeExpansion implements PawNodeExpansion
{
   protected final PawPlannerParametersReadOnly parameters;
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;

   public ParameterBasedPawNodeExpansion(PawPlannerParametersReadOnly parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.parameters = parameters;
      this.xGaitSettings = xGaitSettings;
   }

   @Override
   public HashSet<PawNode> expandNode(PawNode node)
   {
      HashSet<PawNode> expansion = new HashSet<>();
      addDefaultPawNodes(node, expansion, PawNode.gridSizeXY);

      return expansion;
   }

   protected void addDefaultPawNodes(PawNode nodeToExpand, HashSet<PawNode> neighboringNodesToPack, double resolution)
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

      Vector2D clearanceVector = new Vector2D(parameters.getMinXClearanceFromPaw(), parameters.getMinYClearanceFromPaw());
      previousNodeOrientation.transform(clearanceVector);

      boolean isMovingFront = movingQuadrant.isQuadrantInFront();
      double maxReach = isMovingFront ? parameters.getMaximumFrontStepReach() : parameters.getMaximumHindStepReach();

      double maxForwardSpeed = xGaitSettings.getMaxSpeed();
      double maxForwardDisplacement = durationBetweenSteps * maxForwardSpeed;
      double maxLateralDisplacement = xGaitSettings.getMaxHorizontalSpeedFraction() * maxForwardDisplacement;
      double maxYawDisplacement = xGaitSettings.getMaxYawSpeedFraction() * maxForwardDisplacement;

      double maxLength = maxForwardDisplacement + parameters.getMaximumDeviationFromXGaitDuringExpansion();
      double maxWidth = maxLateralDisplacement + parameters.getMaximumDeviationFromXGaitDuringExpansion();

      Vector2D nominalPawOffset = new Vector2D(0.5 * nextQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength()),
                                                0.5 * nextQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth()));
      Point2D newXGaitPosition = new Point2D();

      double maxNegativeYaw = movingQuadrant.isQuadrantOnLeftSide() ? parameters.getMaximumStepYawInward() : -parameters.getMaximumStepYawOutward();
      double maxPositiveYaw = movingQuadrant.isQuadrantOnLeftSide() ? parameters.getMaximumStepYawOutward() : -parameters.getMaximumStepYawInward();

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

//            double absoluteMaxYawDisplacement = maxYawDisplacement;
            double absoluteMaxYawDisplacement = InterpolationTools.hermiteInterpolate(maxYawDisplacement, 0.25 * maxYawDisplacement, translation / maxReach);
            double minYaw = PawNode.snapToYawGrid(Math.max(maxNegativeYaw, -absoluteMaxYawDisplacement)) * PawNode.gridSizeYaw;
            double maxYaw = PawNode.snapToYawGrid(Math.min(maxPositiveYaw, absoluteMaxYawDisplacement)) * PawNode.gridSizeYaw;

            Vector2D movingVector = new Vector2D(movingX, movingY);
            previousNodeOrientation.transform(movingVector);

            newXGaitPosition.add(previousXGaitCenterPoint, movingVector);

            for (double yaw = minYaw; yaw <= maxYaw; yaw += PawNode.gridSizeYaw)
            {
               double newYaw = AngleTools.trimAngleMinusPiToPi(previousYaw + yaw);
               Orientation3DReadOnly newNodeOrientation = new Quaternion(newYaw, 0.0, 0.0);

               Vector2D pawOffset = new Vector2D(nominalPawOffset);
               newNodeOrientation.transform(pawOffset);

               Point2D newNodePosition = new Point2D(newXGaitPosition);
               newNodePosition.add(pawOffset);

               if (!checkNodeIsFarEnoughFromOtherPaws(newNodePosition, clearanceVector, nodeToExpand))
                  continue;

               PawNode offsetNode = constructNodeInPreviousNodeFrame(newNodePosition, newYaw, nodeToExpand, xGaitSettings);

               neighboringNodesToPack.add(offsetNode);
            }
         }
      }
   }

   static boolean checkNodeIsFarEnoughFromOtherPaws(Point2DReadOnly nodePositionToCheck, Vector2DReadOnly requiredClearance, PawNode previousNode)
   {
      RobotQuadrant nextQuadrant = previousNode.getMovingQuadrant().getNextRegularGaitSwingQuadrant();

      for (RobotQuadrant otherQuadrant : RobotQuadrant.values)
      {
         if (nextQuadrant == otherQuadrant)
            continue;

         double otherNodeX = previousNode.getX(otherQuadrant);
         double otherNodeY = previousNode.getY(otherQuadrant);

         if (!checkNodeIsFarEnoughFromOtherPaw(nodePositionToCheck, requiredClearance, otherNodeX, otherNodeY))
            return false;
      }

      return true;
   }

   /**
    * Checks if the node position being added {@param nodePositionToCheck} is too close to another paw position, defined by ({@param otherPawX}, {@param otherPawY})
    *
    */
   static boolean checkNodeIsFarEnoughFromOtherPaw(Point2DReadOnly nodePositionToCheck, Vector2DReadOnly requiredClearance, double otherPawX, double otherPawY)
   {
      double maxForward = otherPawX + Math.abs(requiredClearance.getX());
      double maxBackward = otherPawX - Math.abs(requiredClearance.getX());

      double maxLeft = otherPawY + Math.abs(requiredClearance.getY());
      double maxRight = otherPawY - Math.abs(requiredClearance.getY());

      if (MathTools.intervalContains(nodePositionToCheck.getX(), maxBackward, maxForward, true, true) && MathTools.intervalContains(nodePositionToCheck.getY(),
                                                                                                                                    maxRight, maxLeft, true, true))
         return false;
      else
         return true;
   }

   private static PawNode constructNodeInPreviousNodeFrame(Point2DReadOnly newNodePosition, double newNodeYaw, PawNode previousNode,
                                                           QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      RobotQuadrant nextQuadrant = previousNode.getMovingQuadrant().getNextRegularGaitSwingQuadrant();
      return PawNode.constructNodeFromOtherNode(nextQuadrant, newNodePosition, newNodeYaw, previousNode, xGaitSettings.getStanceLength(),
                                                xGaitSettings.getStanceWidth());
   }
}

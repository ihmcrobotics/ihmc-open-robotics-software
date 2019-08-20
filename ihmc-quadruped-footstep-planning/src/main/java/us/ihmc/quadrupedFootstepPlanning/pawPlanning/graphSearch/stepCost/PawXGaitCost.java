package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PawXGaitCost implements PawNodeCost
{
   private final PawPlannerParameters plannerParameters;
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;

   private final Point2D endXGaitInStartFrame = new Point2D();

   public PawXGaitCost(PawPlannerParameters plannerParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.plannerParameters = plannerParameters;
      this.xGaitSettings = xGaitSettings;
   }

   @Override
   public double compute(PawNode startNode, PawNode endNode)
   {
      RobotQuadrant movingQuadrant = endNode.getMovingQuadrant();
      RobotQuadrant previousQuadrant = startNode.getMovingQuadrant();
      if (movingQuadrant.getNextReversedRegularGaitSwingQuadrant() != previousQuadrant)
      { // fixme this should account for different phases.
         throw new RuntimeException("For some reason the paw movement is out of order.");
      }

      Point2DReadOnly startXGaitPosition = startNode.getOrComputeXGaitCenterPoint();
      Point2DReadOnly endXGaitPosition = endNode.getOrComputeXGaitCenterPoint();

      double durationBetweenSteps = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(previousQuadrant, xGaitSettings);
      double desiredMaxForwardTranslation = durationBetweenSteps * plannerParameters.getMaxWalkingSpeedMultiplier() * xGaitSettings.getMaxSpeed();
      double desiredMaxHorizontalTranslation = xGaitSettings.getMaxHorizontalSpeedFraction() * desiredMaxForwardTranslation;
      double desiredMaxYaw = xGaitSettings.getMaxYawSpeedFraction() * desiredMaxForwardTranslation;

      endXGaitInStartFrame.sub(endXGaitPosition, startXGaitPosition);
      startNode.getStepOrientation().inverseTransform(endXGaitInStartFrame);

      double distanceToNominalVelocityEllipse = EllipseTools
            .getDistanceFromPointToEllipse(desiredMaxForwardTranslation, desiredMaxHorizontalTranslation, endXGaitInStartFrame.getX(),
                                           endXGaitInStartFrame.getY());

      double costOfNominalVelocity = plannerParameters.getDesiredVelocityWeight() * Math.abs(distanceToNominalVelocityEllipse);

      double yawRotation = AngleTools.computeAngleDifferenceMinusPiToPi(startNode.getStepYaw(), endNode.getStepYaw());
      double distancePastMaximumEllipseTranslation = Math.max(distanceToNominalVelocityEllipse, 0.0);
      double distancePastMaximumYawRotation = Math.max(Math.abs(yawRotation) - desiredMaxYaw, 0.0);
      double extraPawTranslationFromRotation = startNode.getNominalStanceLength() * Math.sin(0.5 * distancePastMaximumYawRotation);

      double costOfXGait = plannerParameters.getXGaitWeight() * (extraPawTranslationFromRotation + distancePastMaximumEllipseTranslation);

      return costOfXGait + costOfNominalVelocity;
   }
}

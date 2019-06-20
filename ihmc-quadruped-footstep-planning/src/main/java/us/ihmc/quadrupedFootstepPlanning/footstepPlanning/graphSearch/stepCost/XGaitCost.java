package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedBasics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class XGaitCost implements FootstepCost
{
   private final FootstepPlannerParameters plannerParameters;
   private final FootstepNodeSnapper snapper;
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final QuadrantDependentList<Point3D> startFootPositions = new QuadrantDependentList<>();

   private final NominalVelocityProvider velocityProvider;

   private final Point2D endXGaitInStartFrame = new Point2D();
   private final Vector2D nominalVelocityHeading = new Vector2D();
   private final Vector2D nominalTranslation = new Vector2D();
   private final Point2D nominalXGaitEndPosition = new Point2D();
   private final Point2D nominalFootEndPosition = new Point2D();
   private final Point2D snappedXGaitStartPosition = new Point2D();

   private final Vector2D footOffsetFromXGait = new Vector2D();

   private final LineSegment2D acceptableTranslationLine = new LineSegment2D();
   private final Quaternion startOrientation = new Quaternion();
   private final Quaternion endOrientation = new Quaternion();


   public XGaitCost(FootstepPlannerParameters plannerParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings, FootstepNodeSnapper snapper,
                    NominalVelocityProvider velocityProvider)
   {
      this.plannerParameters = plannerParameters;
      this.xGaitSettings = xGaitSettings;
      this.snapper = snapper;
      this.velocityProvider = velocityProvider;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         startFootPositions.put(robotQuadrant, new Point3D());
   }

   // FIXME make this more efficient
   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      RobotQuadrant movingQuadrant = endNode.getMovingQuadrant();
      RobotQuadrant previousQuadrant = startNode.getMovingQuadrant();
      if (movingQuadrant.getNextReversedRegularGaitSwingQuadrant() != previousQuadrant)
      { // fixme this should account for different phases.
         throw new RuntimeException("For some reason the feet movement is out of order.");
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FootstepNodeSnapData snapData = snapper.snapFootstepNode(startNode.getXIndex(robotQuadrant), startNode.getYIndex(robotQuadrant));
         startFootPositions.get(robotQuadrant).set(startNode.getX(robotQuadrant), startNode.getY(robotQuadrant), 0.0);
         snapData.getSnapTransform().transform(startFootPositions.get(robotQuadrant));
      }

      Point2DReadOnly startXGaitPosition = startNode.getOrComputeXGaitCenterPoint();
      Point2DReadOnly endXGaitPosition = endNode.getOrComputeXGaitCenterPoint();

      double nominalPitch = QuadrupedSupportPolygon.getNominalPitch(startFootPositions, 4);
      startOrientation.setYawPitchRoll(startNode.getStepYaw(), nominalPitch, 0.0);

      nominalVelocityHeading.set(velocityProvider.computeNominalNormalizedVelocityHeadingInWorld(startNode));
      startOrientation.transform(nominalVelocityHeading, false);

      double durationBetweenSteps = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(previousQuadrant, xGaitSettings);
      double desiredMaxForwardTranslation = durationBetweenSteps * plannerParameters.getMaxWalkingSpeedMultiplier() * xGaitSettings.getMaxSpeed();
      double desiredMaxHorizontalTranslation = xGaitSettings.getMaxHorizontalSpeedFraction() * desiredMaxForwardTranslation;
      double desiredMaxYaw = xGaitSettings.getMaxYawSpeedFraction() * desiredMaxForwardTranslation;

      endXGaitInStartFrame.sub(endXGaitPosition, startXGaitPosition);
      startNode.getStepOrientation().inverseTransform(endXGaitInStartFrame);

      double distanceToNominalVelocityEllipse = EllipseTools.getDistanceFromPointToEllipse(desiredMaxForwardTranslation, desiredMaxHorizontalTranslation,
                                                                                              endXGaitInStartFrame.getX(), endXGaitInStartFrame.getY());

      double costOfNominalVelocity = plannerParameters.getDesiredVelocityWeight() * Math.abs(distanceToNominalVelocityEllipse);


      double yawRotation = AngleTools.computeAngleDifferenceMinusPiToPi(startNode.getStepYaw(), endNode.getStepYaw());
      double distancePastMaximumEllipseTranslation = Math.max(distanceToNominalVelocityEllipse, 0.0);
      double distancePastMaximumYawRotation = Math.max(Math.abs(yawRotation) - desiredMaxYaw, 0.0);
      double extraFootTranslationFromRotation = startNode.getNominalStanceLength() * Math.sin(0.5 * distancePastMaximumYawRotation);

      double costOfXGait = plannerParameters.getXGaitWeight() * (extraFootTranslationFromRotation + distancePastMaximumEllipseTranslation);

      return costOfXGait + costOfNominalVelocity;
   }
}

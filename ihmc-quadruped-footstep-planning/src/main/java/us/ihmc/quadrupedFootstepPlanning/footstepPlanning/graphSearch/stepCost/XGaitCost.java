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
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class XGaitCost implements FootstepCost
{
   private final FootstepPlannerParameters plannerParameters;
   private final FootstepNodeSnapper snapper;
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final QuadrantDependentList<Point3D> startFootPositions = new QuadrantDependentList<>();

   private final NominalVelocityProvider velocityProvider;

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
      double desiredMaxForwardSpeed = plannerParameters.getMaxWalkingSpeedMultiplier() * xGaitSettings.getMaxSpeed();
      double desiredMaxHorizontalSpeed = xGaitSettings.getMaxHorizontalSpeedFraction() * desiredMaxForwardSpeed;

      double desiredVelocityAtHeading = EllipseTools.computeMagnitudeOnEllipseInDirection(desiredMaxForwardSpeed, desiredMaxHorizontalSpeed, nominalVelocityHeading.getX(),
                                                                             nominalVelocityHeading.getY());

      nominalTranslation.set(nominalVelocityHeading);
      nominalTranslation.scale(desiredVelocityAtHeading * durationBetweenSteps);
      startOrientation.inverseTransform(nominalTranslation, false);

      nominalXGaitEndPosition.add(startXGaitPosition, nominalTranslation);

      double nominalYawOfEnd = velocityProvider.computeNominalYaw(nominalXGaitEndPosition, startNode.getStepYaw());
      if (Double.isNaN(nominalYawOfEnd))
         nominalYawOfEnd = startNode.getStepYaw();

      endOrientation.setYawPitchRoll(nominalYawOfEnd, 0.0, 0.0);

      footOffsetFromXGait.set(0.5 * movingQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength()),
                              0.5 * movingQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth()));
      endOrientation.transform(footOffsetFromXGait, false);

      nominalFootEndPosition.add(nominalXGaitEndPosition, footOffsetFromXGait);

      FootstepNode.snapPointToGrid(nominalFootEndPosition);

      double costOfNominalVelocity = plannerParameters.getDesiredVelocityWeight() * EuclidCoreTools
            .norm(endNode.getX(movingQuadrant) - nominalFootEndPosition.getX(), endNode.getY(movingQuadrant) - nominalFootEndPosition.getY());

      FootstepNode.snapPointToGrid(nominalXGaitEndPosition);
      FootstepNode.snapPointToGrid(startXGaitPosition, snappedXGaitStartPosition);

      double distanceFromNominalXGaitCenter;
      if (nominalXGaitEndPosition.distance(startXGaitPosition) < 1e-2)
      {
         distanceFromNominalXGaitCenter = endXGaitPosition.distance(snappedXGaitStartPosition);
      }
      else
      {
         acceptableTranslationLine.set(snappedXGaitStartPosition, nominalXGaitEndPosition);
         distanceFromNominalXGaitCenter = acceptableTranslationLine.distance(endXGaitPosition);
      }
      double costOfXGait = plannerParameters.getXGaitWeight() * (distanceFromNominalXGaitCenter);

      return costOfNominalVelocity + costOfXGait;
   }
}

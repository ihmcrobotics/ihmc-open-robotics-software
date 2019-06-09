package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedBasics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class XGaitCost implements FootstepCost
{
   private final FootstepPlannerParameters plannerParameters;
   private final FootstepNodeSnapper snapper;
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final QuadrantDependentList<Point3D> startFootPositions = new QuadrantDependentList<>();

   private final NominalVelocityProvider velocityProvider;


   public XGaitCost(FootstepPlannerParameters plannerParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings, FootstepNodeSnapper snapper, NominalVelocityProvider velocityProvider)
   {
      this.plannerParameters = plannerParameters;
      this.xGaitSettings = xGaitSettings;
      this.snapper = snapper;
      this.velocityProvider = velocityProvider;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         startFootPositions.put(robotQuadrant, new Point3D());
   }


   private final FramePose3D startXGaitPose = new FramePose3D();
   private final FramePose3D endXGaitPose = new FramePose3D();
   private final PoseReferenceFrame startXGaitPoseFrame = new PoseReferenceFrame("startXGaitPose", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame endXGaitPoseFrame = new PoseReferenceFrame("endXGaitPose", ReferenceFrame.getWorldFrame());

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



      double nominalPitch = QuadrupedSupportPolygon.getNominalPitch(startFootPositions, 4);
      startXGaitPose.setPosition(startNode.getOrComputeXGaitCenterPoint());
      startXGaitPose.setOrientationYawPitchRoll(startNode.getNominalYaw(), nominalPitch, 0.0);
      startXGaitPoseFrame.setPoseAndUpdate(startXGaitPose);


      FrameVector2D nominalVelocityHeading = new FrameVector2D(worldFrame, velocityProvider.computeNominalVelocityHeadingInWorld(startNode));
      nominalVelocityHeading.changeFrameAndProjectToXYPlane(startXGaitPoseFrame);
      nominalVelocityHeading.normalize();


      double durationBetweenSteps = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(previousQuadrant, xGaitSettings);
      double desiredMaxForwardSpeed = plannerParameters.getMaxWalkingSpeedMultiplier() * xGaitSettings.getMaxSpeed();
      double desiredMaxHorizontalSpeed = xGaitSettings.getMaxHorizontalSpeedFraction() * desiredMaxForwardSpeed;

      FrameVector2D desiredVelocity = new FrameVector2D(nominalVelocityHeading);
      desiredVelocity.setX(desiredVelocity.getX() * desiredMaxForwardSpeed);
      desiredVelocity.setY(desiredVelocity.getY() * desiredMaxHorizontalSpeed);
      desiredVelocity.changeFrameAndProjectToXYPlane(worldFrame);

      FramePoint2D endXGaitPosition = new FramePoint2D(desiredVelocity);
      endXGaitPosition.scale(durationBetweenSteps);
      endXGaitPosition.add(startNode.getOrComputeXGaitCenterPoint());

      double nominalYawOfEnd = velocityProvider.computeNominalYaw(endXGaitPosition);
      if (Double.isNaN(nominalYawOfEnd))
         nominalYawOfEnd = startNode.getNominalYaw();

      if (Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(startNode.getNominalYaw(), nominalYawOfEnd)) > Math.PI / 2.0) // greater than 90 degrees, so go backwards
      {
         nominalYawOfEnd = AngleTools.trimAngleMinusPiToPi(nominalYawOfEnd + Math.PI);
      }


         endXGaitPose.setPosition(endXGaitPosition);
      endXGaitPose.setOrientationYawPitchRoll(nominalYawOfEnd, 0.0, 0.0);
      endXGaitPoseFrame.setPoseAndUpdate(endXGaitPose);

      FramePoint2D nominalEndFootPosition = new FramePoint2D(endXGaitPoseFrame);
      nominalEndFootPosition.setX(0.5 * movingQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength()));
      nominalEndFootPosition.setY(0.5 * movingQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth()));
      nominalEndFootPosition.changeFrameAndProjectToXYPlane(worldFrame);

      Point2D endFootPosition = new Point2D(endNode.getX(movingQuadrant), endNode.getY(movingQuadrant));

      return plannerParameters.getXGaitWeight() * endFootPosition.distanceSquared(nominalEndFootPosition);
   }
}

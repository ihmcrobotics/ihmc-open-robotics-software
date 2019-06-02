package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedBasics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
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
   private final PoseReferenceFrame startXGaitPoseFrame = new PoseReferenceFrame("startXGaitPose", ReferenceFrame.getWorldFrame());

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

      Vector2D desiredVelocity = new Vector2D();
      desiredVelocity.setX(nominalVelocityHeading.getX() * desiredMaxForwardSpeed);
      desiredVelocity.setY(nominalVelocityHeading.getY() * desiredMaxHorizontalSpeed);

      FramePoint2D edgeVelocity = new FramePoint2D(worldFrame, endNode.getX(movingQuadrant), endNode.getY(movingQuadrant));

      edgeVelocity.changeFrameAndProjectToXYPlane(startXGaitPoseFrame);
      edgeVelocity.addX(0.5 * movingQuadrant.getEnd().negateIfFrontEnd(xGaitSettings.getStanceLength()));
      edgeVelocity.addY(0.5 * movingQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth()));
      edgeVelocity.scale(1.0 / durationBetweenSteps);

      return plannerParameters.getXGaitWeight() * EuclidCoreTools.norm(desiredVelocity.getX() - edgeVelocity.getX(), desiredVelocity.getY() - edgeVelocity.getY());
   }
}

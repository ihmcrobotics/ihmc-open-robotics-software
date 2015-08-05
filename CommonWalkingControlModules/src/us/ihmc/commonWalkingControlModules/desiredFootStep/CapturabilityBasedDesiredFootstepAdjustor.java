package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.captureRegion.SteppingStonesCaptureRegionIntersectionCalculator;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.simulationconstructionset.util.ground.steppingStones.SteppingStones;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;

public class CapturabilityBasedDesiredFootstepAdjustor implements DesiredFootstepAdjustor
{
   private final CouplingRegistry couplingRegistry;

   private final YoVariableRegistry registry = new YoVariableRegistry("CapturabilityBasedDesiredFootstepCalculator");

   // Stepping Stones if the world isn't flat:
   private final SteppingStonesCaptureRegionIntersectionCalculator steppingStonesCaptureRegionIntersectionCalculator;

   private final BooleanYoVariable projectIntoCaptureRegion = new BooleanYoVariable("projectIntoCaptureRegion",
         "Whether or not to project the step into the capture region", registry);

   private final BooleanYoVariable nextStepIsInsideCaptureRegion = new BooleanYoVariable("nextStepIsInsideCaptureRegion", registry);

   private final SideDependentList<? extends ContactablePlaneBody> feet;

   private double footForwardOffset;
   private double footBackwardOffset;
   private double footWidth;

   public CapturabilityBasedDesiredFootstepAdjustor(CouplingRegistry couplingRegistry, SideDependentList<? extends ContactablePlaneBody> feet,
         SteppingStones steppingStones, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.couplingRegistry = couplingRegistry;
      this.feet = feet;

      if (steppingStones != null)
      {
         steppingStonesCaptureRegionIntersectionCalculator = new SteppingStonesCaptureRegionIntersectionCalculator(steppingStones, registry,
               yoGraphicsListRegistry);
      }
      else
      {
         steppingStonesCaptureRegionIntersectionCalculator = null;
      }

      parentRegistry.addChild(registry);
   }

   public Footstep adjustDesiredFootstep(Footstep stanceFootstep, Footstep baseSwingFootstep)
   {
      if (!projectIntoCaptureRegion.getBooleanValue())
      {
         nextStepIsInsideCaptureRegion.set(false);

         return baseSwingFootstep;
      }

      FrameConvexPolygon2d captureRegion = couplingRegistry.getCaptureRegion();

      boolean noCaptureRegion = captureRegion == null || captureRegion.isEmpty();
      if (noCaptureRegion)
      {
         nextStepIsInsideCaptureRegion.set(false);

         return baseSwingFootstep;
      }

      RobotSide swingLegSide = baseSwingFootstep.getRobotSide();

      if (steppingStonesCaptureRegionIntersectionCalculator != null)
      {
         return projectIntoSteppingStonesAndCaptureRegion(baseSwingFootstep, swingLegSide, captureRegion);
      }
      else
      {
         return projectIntoCaptureRegion(baseSwingFootstep, swingLegSide, captureRegion);
      }
   }

   private Footstep projectIntoCaptureRegion(Footstep baseFootstep, RobotSide swingLegSide, FrameConvexPolygon2d captureRegion)
   {
      FramePoint2d nextStep2d = new FramePoint2d();
      baseFootstep.getPosition2d(nextStep2d);
      nextStep2d.changeFrame(captureRegion.getReferenceFrame());
      FrameConvexPolygon2d nextStepFootPolygon = buildNextStepFootPolygon(nextStep2d);

      if (nextStepFootPolygon.intersectionWith(captureRegion) == null)
      {
         nextStepIsInsideCaptureRegion.set(false);

         nextStep2d.changeFrame(captureRegion.getReferenceFrame());
         captureRegion.orthogonalProjection(nextStep2d);
         nextStep2d.changeFrame(baseFootstep.getParentFrame());
         baseFootstep.setPositionChangeOnlyXY(nextStep2d);
      }
      else
      {
         nextStepIsInsideCaptureRegion.set(true);
      }

      return baseFootstep;
   }

   private Footstep projectIntoSteppingStonesAndCaptureRegion(Footstep baseFootstep, RobotSide swingLegSide, FrameConvexPolygon2d captureRegion)
   {
      captureRegion.changeFrame(ReferenceFrame.getWorldFrame());
      ArrayList<ConvexPolygon2d> steppingStoneCaptureRegionIntersections = steppingStonesCaptureRegionIntersectionCalculator
            .findIntersectionsBetweenSteppingStonesAndCaptureRegion(captureRegion);

      FramePoint2d nextStep2d = new FramePoint2d();
      baseFootstep.getPosition2d(nextStep2d);
      nextStep2d.changeFrame(captureRegion.getReferenceFrame());

      Point2d oldLocation = nextStep2d.getPointCopy();
      Point2d newLocation = computeBestNearestPointToStepTo(oldLocation, steppingStoneCaptureRegionIntersections);

      if (newLocation.distance(oldLocation) < 0.002)
      {
         nextStepIsInsideCaptureRegion.set(true);
      }

      else
      {
         nextStepIsInsideCaptureRegion.set(false);

         nextStep2d.setX(newLocation.getX());
         nextStep2d.setY(newLocation.getY());

         nextStep2d.changeFrame(baseFootstep.getParentFrame());

         baseFootstep.setPositionChangeOnlyXY(nextStep2d);
      }

      return baseFootstep;
   }

   private FrameConvexPolygon2d buildNextStepFootPolygon(FramePoint2d nextStep) // TODO: doesn't account for foot yaw
   {
      ArrayList<FramePoint2d> nextStepFootPolygonPoints = new ArrayList<FramePoint2d>(4);

      FramePoint2d nextStepFrontRightCorner = new FramePoint2d(nextStep);
      FramePoint2d nextStepFrontLeftCorner = new FramePoint2d(nextStep);
      FramePoint2d nextStepBackRightCorner = new FramePoint2d(nextStep);
      FramePoint2d nextStepBackLeftCorner = new FramePoint2d(nextStep);

      FramePoint2d tempNextStepOffset = new FramePoint2d(nextStep.getReferenceFrame(), 0.0, 0.0);

      // Front right corner
      tempNextStepOffset.set(footForwardOffset, -footWidth / 2.0);
      nextStepFrontRightCorner.add(tempNextStepOffset);
      nextStepFootPolygonPoints.add(nextStepFrontRightCorner);

      // Front left corner
      tempNextStepOffset.set(footForwardOffset, footWidth / 2.0);
      nextStepFrontLeftCorner.add(tempNextStepOffset);
      nextStepFootPolygonPoints.add(nextStepFrontLeftCorner);

      // Back right corner
      tempNextStepOffset.set(-footBackwardOffset, -footWidth / 2.0);
      nextStepBackRightCorner.add(tempNextStepOffset);
      nextStepFootPolygonPoints.add(nextStepBackRightCorner);

      // Back left left corner
      tempNextStepOffset.set(-footBackwardOffset, footWidth / 2.0);
      nextStepBackLeftCorner.add(tempNextStepOffset);
      nextStepFootPolygonPoints.add(nextStepBackLeftCorner);

      FrameConvexPolygon2d nextStepFootPolygon = new FrameConvexPolygon2d(nextStepFootPolygonPoints);

      return nextStepFootPolygon;
   }

   private Point2d computeBestNearestPointToStepTo(Point2d nominalLocation2d, ArrayList<ConvexPolygon2d> captureRegionSteppingStonesIntersections)
   {
      Point2d nearestPoint = new Point2d();
      double nearestDistanceSquared = Double.POSITIVE_INFINITY;
      Point2d pointToTest = new Point2d();

      if (captureRegionSteppingStonesIntersections != null) // If there are no captureRegionSteppingStonesIntersections, just keep stepping where you were before for now...
      {
         for (ConvexPolygon2d possiblePlaceToStep : captureRegionSteppingStonesIntersections)
         {
            pointToTest.set(nominalLocation2d);
            possiblePlaceToStep.orthogonalProjection(pointToTest);

            double possibleDistanceSquared = nominalLocation2d.distanceSquared(pointToTest);

            if (possibleDistanceSquared < nearestDistanceSquared)
            {
               nearestPoint.set(pointToTest);
               nearestDistanceSquared = possibleDistanceSquared;
            }
         }

         if (nearestDistanceSquared != Double.POSITIVE_INFINITY) // If there are no near centroids, just keep stepping where you were before...
         {
            return nearestPoint;
         }
      }

      return nominalLocation2d;
   }

   public void setUpParametersForR2(double footBackwardOffset, double footForwardOffset, double footWidth)
   {
      this.footBackwardOffset = footBackwardOffset;
      this.footForwardOffset = footForwardOffset;
      this.footWidth = footWidth;

      projectIntoCaptureRegion.set(true);
   }

   public void setupParametersForM2V2(double footBackwardOffset, double footForwardOffset, double footWidth)
   {
      this.footBackwardOffset = footBackwardOffset;
      this.footForwardOffset = footForwardOffset;
      this.footWidth = footWidth;

      projectIntoCaptureRegion.set(true);
   }
}
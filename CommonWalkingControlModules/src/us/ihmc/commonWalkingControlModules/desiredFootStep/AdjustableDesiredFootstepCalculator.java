package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.captureRegion.SteppingStonesCaptureRegionIntersectionCalculator;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.ConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.ground.steppingStones.SteppingStones;


public class AdjustableDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   private final boolean CHECK_STEP_INTO_CAPTURE_REGION = true; //false;

   private final YoVariableRegistry registry = new YoVariableRegistry("VelocityControlDesiredStepLocationCalculator");

   // Tunable robot-dependent parameters
   private double goodStandingStepWidth;
   private double goodWalkingStepWidth;

   private double robotMaxVelocity;
   private double robotMinVelocity;

   private double insideStepAdjustmentForTurning;
   private double outsideStepAdjustmentForTurning;

   private double stepWidthSlopeProfile;
   private double stepWidthYInterceptProfile;

   private double footForwardOffset;
   private double footBackwardOffset;
   private double footWidth;

   // Stepping Stones if the world isn't flat:
   private final SteppingStonesCaptureRegionIntersectionCalculator steppingStonesCaptureRegionIntersectionCalculator;
   
   // Footstep parameters
   private final DoubleYoVariable stepLength = new DoubleYoVariable("stepLength", "step length. [m]", registry);
   private final DoubleYoVariable stepWidth = new DoubleYoVariable("stepWidth", "step width. [m]", registry);
   private final DoubleYoVariable stepHeight = new DoubleYoVariable("stepHeight", "step height. [m]", registry);
   private final DoubleYoVariable stepYaw = new DoubleYoVariable("stepYaw", "step yaw. [rad]", registry);
   private final DoubleYoVariable stepPitch = new DoubleYoVariable("stepPitch", "step pitch. [rad]", registry);
   private final DoubleYoVariable stepRoll = new DoubleYoVariable("stepRoll", "step roll. [rad]", registry);

   private final DoubleYoVariable previousStepLength = new DoubleYoVariable("previousStepLength", registry);
   private final DoubleYoVariable minimalStepLength = new DoubleYoVariable("minimalStepLength", registry);

   // Gain
   private final DoubleYoVariable kpStepLength = new DoubleYoVariable("kpStepLength", registry);
   private final DoubleYoVariable KpStepYaw = new DoubleYoVariable("KpStepYaw", registry);

   // Method variables
   private final DoubleYoVariable headingCross = new DoubleYoVariable("headingCross", "Mag of cross product from DesiredHeading to FinalHeading", registry);
   private final DoubleYoVariable stepLengthForDesiredVelocity = new DoubleYoVariable("stepLengthForDesiredVelocity", registry);
   private final BooleanYoVariable nextStepIsInsideCaptureRegion = new BooleanYoVariable("nextStepIsInsideCaptureRegion", registry);;

   // Shared data
   private final CouplingRegistry couplingRegistry;
   private final CommonWalkingReferenceFrames referenceFrames;

   // Control modules
   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final DesiredVelocityControlModule desiredVelocityControlModule;

   // Desired Footstep
   // private Footstep initialFootstep;
   private FramePoint desiredFootstepPosition;
   private Orientation desiredFootstepOrientation;
   private Footstep desiredFootstep;

   // Constructor
   public AdjustableDesiredFootstepCalculator(CouplingRegistry couplingRegistry, DesiredHeadingControlModule desiredHeadingControlModule,
         DesiredVelocityControlModule desiredVelocityControlModule, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, CommonWalkingReferenceFrames referenceFrames)
   {
      this(null, couplingRegistry, desiredHeadingControlModule,
            desiredVelocityControlModule, parentRegistry, dynamicGraphicObjectsListRegistry, referenceFrames);
   }
   
   public AdjustableDesiredFootstepCalculator(SteppingStones steppingStones, CouplingRegistry couplingRegistry, DesiredHeadingControlModule desiredHeadingControlModule,
           DesiredVelocityControlModule desiredVelocityControlModule, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, CommonWalkingReferenceFrames referenceFrames)
   {
	   parentRegistry.addChild(registry);
	   
	   if (steppingStones != null)
      {
         steppingStonesCaptureRegionIntersectionCalculator = new SteppingStonesCaptureRegionIntersectionCalculator(steppingStones, registry,
                 dynamicGraphicObjectsListRegistry);
      }
      else
      {
         steppingStonesCaptureRegionIntersectionCalculator = null;
      }

	   
      this.couplingRegistry = couplingRegistry;
      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.desiredVelocityControlModule = desiredVelocityControlModule;
      this.referenceFrames = referenceFrames;     
   }

   // Getters
   public Footstep getDesiredFootstep()
   {
	   return desiredFootstep;
   }

   // Initialize the Desired Footstep
   public void initializeDesiredFootstep(RobotSide swingLegSide)
   {
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();

      // Step Position
      computeDesiredFootstepPosition(swingLegSide, desiredHeadingFrame);    

      // Step Orientation
      computeFootstepOrientation(swingLegSide, desiredHeadingFrame);
      
      // Adjust position if required and update the desired footstep
      updateDesiredFootstep(swingLegSide.getOppositeSide());      
   }

   private void computeDesiredFootstepPosition(RobotSide swingLegSide, ReferenceFrame desiredHeadingFrame)
   {
      RobotSide supportLegSide = swingLegSide.getOppositeSide();

      // Compute previous Step Length
      computePreviousStepLength(desiredHeadingFrame);

      // Compute Step Length
      computeStepLength(swingLegSide);

      // Compute Width
      computeStepWidth(swingLegSide, desiredHeadingFrame);

      // Create the desired footstep position using the parameters previously computed
      ReferenceFrame supportLegAnkleZUpFrame = referenceFrames.getAnkleZUpReferenceFrames().get(supportLegSide);
      desiredFootstepPosition = new FramePoint(supportLegAnkleZUpFrame, stepLength.getDoubleValue(), stepWidth.getDoubleValue(), stepHeight.getDoubleValue());
      
//      desiredFootstepPosition.changeFrame(supportLegAnkleZUpFrame);
   }

   private void computePreviousStepLength(ReferenceFrame desiredHeadingFrame)
   {
      FramePoint leftFoot = new FramePoint(referenceFrames.getAnkleZUpFrame(RobotSide.LEFT));
      leftFoot.changeFrame(referenceFrames.getAnkleZUpFrame(RobotSide.RIGHT));
      FrameVector footToFoot = new FrameVector(leftFoot);
      footToFoot.changeFrame(desiredHeadingFrame);     
      previousStepLength.set(Math.abs(footToFoot.getX()));
   }

   private void computeStepLength(RobotSide swingLegSide)
   {
      double swingDuration = couplingRegistry.getSingleSupportDuration();
      double stanceDuration = couplingRegistry.getDoubleSupportDuration();
      double totalDuration = swingDuration + stanceDuration;

      // Do not multiply by 2, this will give you the stride length and not the step length !!!
      stepLengthForDesiredVelocity.set(totalDuration * desiredVelocityControlModule.getDesiredVelocity().getX());

      double stepLengthError = stepLengthForDesiredVelocity.getDoubleValue() - stepLength.getDoubleValue();

      stepLength.set(previousStepLength.getDoubleValue() + stepLengthError * kpStepLength.getDoubleValue());
      if (stepLength.getDoubleValue() < minimalStepLength.getDoubleValue())
         stepLength.set(minimalStepLength.getDoubleValue());
   }

   private void computeStepWidth(RobotSide swingLegSide, ReferenceFrame desiredHeadingFrame)
   {
      double tempStepWidth = stepWidthSlopeProfile * couplingRegistry.getDesiredVelocity().length() + stepWidthYInterceptProfile;
      MathTools.clipToMinMax(tempStepWidth, goodWalkingStepWidth, goodStandingStepWidth);
      stepWidth.set(swingLegSide.negateIfRightSide(tempStepWidth));

      stepWidth.set(stepWidth.getDoubleValue() + getStepWidthAdjustmentForTurning(swingLegSide, desiredHeadingFrame));
   }

   private double getStepWidthAdjustmentForTurning(RobotSide swingLegSide, ReferenceFrame desiredHeadingFrame)
   {
      // This is where we want to adjust the width depending on "inside" or "outside" foot
      FrameVector desiredHeading = new FrameVector(desiredHeadingFrame, 1.0, 0.0, 0.0);
      FrameVector finalHeading = desiredHeadingControlModule.getFinalHeadingTarget().changeFrameCopy(desiredHeadingFrame);
      FrameVector crossVector = new FrameVector(desiredHeading);
      crossVector.cross(desiredHeading, finalHeading);
      double magCrossProductFromDesiredToFinalHeading = crossVector.getZ();
      headingCross.set(magCrossProductFromDesiredToFinalHeading);

      double threshholdForStepWidthAdjustment = 0.00;

      if (magCrossProductFromDesiredToFinalHeading > threshholdForStepWidthAdjustment)
      {
         if (swingLegSide == RobotSide.LEFT)
            return insideStepAdjustmentForTurning;
         else
            return outsideStepAdjustmentForTurning;
      }
      else if (magCrossProductFromDesiredToFinalHeading < -threshholdForStepWidthAdjustment)
      {
         if (swingLegSide == RobotSide.LEFT)
            return -outsideStepAdjustmentForTurning;
         else
            return -insideStepAdjustmentForTurning;
      }
      else
         return 0.0;
   }

   private void computeFootstepOrientation(RobotSide swingLegSide, ReferenceFrame desiredHeadingFrame)
   {
      // Compute Step yaw
      computeFootstepYaw(swingLegSide, desiredHeadingFrame);

      // Compute Step pitch
      computeFootstepPitch();

      // Compute Step roll
      computeFootstepRoll();
      
      // Create the desired footstep orientation using the parameters previously computed
      desiredFootstepOrientation = new Orientation(desiredFootstepPosition.getReferenceFrame(), stepYaw.getDoubleValue(), stepPitch.getDoubleValue(), stepRoll.getDoubleValue());
   }

   private void computeFootstepYaw(RobotSide swingSide, ReferenceFrame desiredHeadingFrame)
   {
      // Compute the difference between the desired Heading frame and the frame of the adjusted footstep position (yaw should be w.r.t. this frame!).
      ReferenceFrame footstepPositionFrame = desiredFootstepPosition.getReferenceFrame();
      Transform3D headingToFootstepPositionTransform = desiredHeadingFrame.getTransformToDesiredFrame(footstepPositionFrame);
      Matrix3d headingToSwingRotation = new Matrix3d();
      headingToFootstepPositionTransform.get(headingToSwingRotation);
      stepYaw.set(RotationFunctions.getYaw(headingToSwingRotation));
   }


   private void computeFootstepPitch()
   {
   }


   private void computeFootstepRoll()
   {
   }


   public void updateDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      adjustDesiredFootstepPosition(swingLegSide, couplingRegistry.getCaptureRegion());
      
      // Assemble Position and Orientation in the desiredFootStep
      FramePose footstepPose = new FramePose(desiredFootstepPosition, desiredFootstepOrientation);
      desiredFootstep = new Footstep(supportLegSide, footstepPose);
   }

   public void adjustDesiredFootstepPosition(RobotSide swingLegSide, FrameConvexPolygon2d captureRegion)
   {
      if (desiredFootstepPosition == null) return; // if not initialized at start of swing first, happens e.g. when balancing on one leg

      // Check if Step Position is inside the Capture Region, Otherwise project step position into the CR
      if (!CHECK_STEP_INTO_CAPTURE_REGION)
      {
         nextStepIsInsideCaptureRegion.set(false);

         return;
      }

      boolean noCaptureRegion = captureRegion == null;
      if (noCaptureRegion)
      {
         nextStepIsInsideCaptureRegion.set(false);
         
         // leave adjusted as it is; not much better you can do
         return;
      }
      
      if (steppingStonesCaptureRegionIntersectionCalculator != null)
      {
         projectIntoSteppingStonesAndCaptureRegion(captureRegion);
      }
      else
      {
         projectIntoCaptureRegion(captureRegion);
      }
      
      
   }
   
   private void projectIntoCaptureRegion(FrameConvexPolygon2d captureRegion)
   {
      FramePoint2d nextStep2d = desiredFootstepPosition.toFramePoint2d();
      nextStep2d.changeFrame(captureRegion.getReferenceFrame());
      FrameConvexPolygon2d nextStepFootPolygon = buildNextStepFootPolygon(nextStep2d);


      if (nextStepFootPolygon.intersectionWith(captureRegion) == null)
      {
         nextStepIsInsideCaptureRegion.set(false);

         captureRegion.orthogonalProjection(nextStep2d);
         desiredFootstepPosition.setX(nextStep2d.getX());
         desiredFootstepPosition.setY(nextStep2d.getY());
      }
      else
      {
         nextStepIsInsideCaptureRegion.set(true);
      }
   }
   
   private void projectIntoSteppingStonesAndCaptureRegion(FrameConvexPolygon2d captureRegion)
   {
      captureRegion = captureRegion.changeFrameCopy(ReferenceFrame.getWorldFrame());
      ArrayList<ConvexPolygon2d> steppingStoneCaptureRegionIntersections = steppingStonesCaptureRegionIntersectionCalculator.findIntersectionsBetweenSteppingStonesAndCaptureRegion(captureRegion);
      
      FramePoint2d nextStep2d = desiredFootstepPosition.toFramePoint2d();
      nextStep2d.changeFrame(captureRegion.getReferenceFrame());
//      FrameConvexPolygon2d nextStepFootPolygon = buildNextStepFootPolygon(nextStep2d);

      Point2d oldLocation = nextStep2d.getPointCopy();
      Point2d newLocation = computeBestNearestPointToStepTo(oldLocation,  steppingStoneCaptureRegionIntersections);
      
      if (newLocation.distance(oldLocation) < 0.002)
      {
         nextStepIsInsideCaptureRegion.set(true);
      }

      else
      {
         nextStepIsInsideCaptureRegion.set(false);

         nextStep2d.setX(newLocation.getX());
         nextStep2d.setY(newLocation.getY());
         
         nextStep2d.changeFrame(desiredFootstepPosition.getReferenceFrame());

         desiredFootstepPosition.setX(nextStep2d.getX());
         desiredFootstepPosition.setY(nextStep2d.getY());
      }
     
   }
  
   
   private Point2d computeBestNearestPointToStepTo(Point2d nominalLocation2d, ArrayList<ConvexPolygon2d> captureRegionSteppingStonesIntersections)
   {
      Point2d nearestPoint = new Point2d();
      double nearestDistanceSquared = Double.POSITIVE_INFINITY;
      Point2d pointToTest = new Point2d();

      if (captureRegionSteppingStonesIntersections != null)    // If there are no captureRegionSteppingStonesIntersections, just keep stepping where you were before for now...
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

         if (nearestDistanceSquared != Double.POSITIVE_INFINITY)    // If there are no near centroids, just keep stepping where you were before...
         {
            return nearestPoint;
//            adjustedStepPosition.set(nearestPoint.x, nearestPoint.y, 0.0);
         }
      }
      
      return nominalLocation2d;
   }
   
   
   

   private FrameConvexPolygon2d buildNextStepFootPolygon(FramePoint2d nextStep)    // TODO: doesn't account for foot yaw
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

   public void setUpParametersForR2(double footBackwardOffset, double footForwardOffset, double footWidth)
   {
      goodStandingStepWidth = (0.1016 + 0.09) * 2.0;
      goodWalkingStepWidth = 0.3272;    // 0.3272 =  minimum metabolic cost @ 0.12*Leg Length(1.245) + 2*half foot width (17.78)

      robotMaxVelocity = 0.60;
      robotMinVelocity = 0.10;

      minimalStepLength.set(0.4);

      computeStepWidthLinearProfile();

      stepPitch.set(-0.3);

      insideStepAdjustmentForTurning = 0.0;
      outsideStepAdjustmentForTurning = 0.0;

      this.footBackwardOffset = footBackwardOffset;
      this.footForwardOffset = footForwardOffset;
      this.footWidth = footWidth;

      kpStepLength.set(0.5);
      KpStepYaw.set(1.0);
   }

   public void setupParametersForM2V2(double footBackwardOffset, double footForwardOffset, double footWidth)
   {
      goodStandingStepWidth = 0.25;
      goodWalkingStepWidth = goodStandingStepWidth - 0.05;

      robotMaxVelocity = 0.5;
      robotMinVelocity = 0.1;

      computeStepWidthLinearProfile();

      stepPitch.set(-0.25);

      insideStepAdjustmentForTurning = 0.0;
      outsideStepAdjustmentForTurning = 0.0;

      this.footBackwardOffset = footBackwardOffset;
      this.footForwardOffset = footForwardOffset;
      this.footWidth = footWidth;
      
      kpStepLength.set(0.5);
      KpStepYaw.set(0.5);
   }

   public void computeStepWidthLinearProfile()
   {
      stepWidthSlopeProfile = (goodStandingStepWidth - goodWalkingStepWidth) / (robotMinVelocity - robotMaxVelocity);
      stepWidthYInterceptProfile = goodStandingStepWidth - stepWidthSlopeProfile * robotMinVelocity;
   }


}

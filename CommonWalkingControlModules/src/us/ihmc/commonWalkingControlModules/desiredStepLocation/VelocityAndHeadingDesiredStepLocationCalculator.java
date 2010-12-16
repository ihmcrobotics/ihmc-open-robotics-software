package us.ihmc.commonWalkingControlModules.desiredStepLocation;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;


public class VelocityAndHeadingDesiredStepLocationCalculator implements DesiredStepLocationCalculator
{
   private final boolean CHECK_STEP_INTO_CAPTURE_REGION;
   private final BooleanYoVariable swingFootIsInsideCaptureRegion;
   private final BooleanYoVariable nextStepIsInsideCaptureRegion;

   // Tunable robot-dependent parameters
   private double goodStandingStepWidth;
   private double goodWalkingStepWidth;

   private double robotMaxVelocity;
   private double robotMinVelocity;

   private double insideStepAdjustmentForTurning;
   private double outsideStepAdjustmentForTurning;

   // Foot dimensions
   private final double footForwardOffset;
   private final double footBackwardOffset;
   private final double footWidth;

   private final YoVariableRegistry registry = new YoVariableRegistry("VelocityControlDesiredStepLocationCalculator");
   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final DesiredVelocityControlModule desiredVelocityControlModule;
   private final CommonWalkingReferenceFrames referenceFrames;

   private final DoubleYoVariable stepLength = new DoubleYoVariable("stepLength", "User determined step length. [m]", registry);
   private final DoubleYoVariable stepWidth = new DoubleYoVariable("stepWidth", "User determined step width. [m]", registry);
   private final DoubleYoVariable stepHeight = new DoubleYoVariable("stepHeight", "User determined step height. [m]", registry);
   private final DoubleYoVariable stepYaw = new DoubleYoVariable("stepYaw", "This is the step yaw. [rad]", registry);

   // Gain
   private final DoubleYoVariable kpStepLength = new DoubleYoVariable("kpStepLength", registry);
   private final DoubleYoVariable KpStepYaw = new DoubleYoVariable("KpStepYaw", registry);

   private final DoubleYoVariable headingCross = new DoubleYoVariable("headingCross", "Mag of cross product from DesiredHeading to FinalHeading", registry);


   private final DoubleYoVariable stepLengthForDesiredVelocity = new DoubleYoVariable("stepLengthForDesiredVelocity", registry);
   private double stepWidthSlopeProfile;
   private double stepWidthYInterceptProfile;

   private FramePoint footstepPosition;

   public VelocityAndHeadingDesiredStepLocationCalculator(DesiredHeadingControlModule desiredHeadingControlModule,
           DesiredVelocityControlModule desiredVelocityControlModule, YoVariableRegistry parentRegistry, CommonWalkingReferenceFrames referenceFrames,
           double footForwardOffset, double footBackwardOffset, double footWidth)
   {
      CHECK_STEP_INTO_CAPTURE_REGION = true;

      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.desiredVelocityControlModule = desiredVelocityControlModule;
      this.referenceFrames = referenceFrames;

      stepLength.set(0.0);
      stepWidth.set(0.0);
      stepHeight.set(0.0);
      stepYaw.set(0.0);

      stepWidthSlopeProfile = 0.0;
      stepWidthYInterceptProfile = 0.0;

      footstepPosition = new FramePoint(desiredHeadingControlModule.getDesiredHeadingFrame());

      swingFootIsInsideCaptureRegion = new BooleanYoVariable("swingFootIsInsideCaptureRegion", parentRegistry);
      nextStepIsInsideCaptureRegion = new BooleanYoVariable("nextStepIsInsideCaptureRegion", parentRegistry);

      this.footForwardOffset = footForwardOffset;
      this.footBackwardOffset = footBackwardOffset;
      this.footWidth = footWidth;

      parentRegistry.addChild(registry);
   }


   public Footstep computeDesiredStepLocation(RobotSide supportLegSide, BipedSupportPolygons bipedSupportPolygons, FrameConvexPolygon2d captureRegion,
           FramePoint capturePoint)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      adjustDesiredStepLocation(swingLegSide, captureRegion);

      return new Footstep(swingLegSide, footstepPosition, stepYaw.getDoubleValue());
   }

   public void initializeAtStartOfSwing(RobotSide swingLegSide, CouplingRegistry couplingRegistry)
   {
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();

      // Step Position
      computeFootstepPosition(swingLegSide, desiredHeadingFrame, couplingRegistry);

      // Check if Step Position is inside the Capture Region, Otherwise project step position into the CR
      adjustDesiredStepLocation(swingLegSide, couplingRegistry.getCaptureRegion());

      // Step Yaw
      computeFootstepYaw(swingLegSide, desiredHeadingFrame);
   }

   public void adjustDesiredStepLocation(RobotSide swingLegSide, FrameConvexPolygon2d captureRegion)
   {
      // Check if Step Position is inside the Capture Region, Otherwise project step position into the CR
      checkIfStepIntersectCaptureRegion(captureRegion, footstepPosition, swingLegSide);
   }



   private void computeFootstepPosition(RobotSide swingLegSide, ReferenceFrame desiredHeadingFrame, CouplingRegistry couplingRegistry)
   {
      RobotSide stanceSide = swingLegSide.getOppositeSide();

      // Compute Step Length
      computeStepLength(swingLegSide, desiredHeadingFrame, couplingRegistry);

      // Compute Width
      computeStepWidth(swingLegSide, desiredHeadingFrame, couplingRegistry);

      // Assign computed values to the desiredStepLocation in the stance
      FrameVector offsetFromFoot = new FrameVector(desiredHeadingFrame, stepLength.getDoubleValue(), stepWidth.getDoubleValue(), stepHeight.getDoubleValue());
      this.footstepPosition = desiredHeadingControlModule.getPositionOffsetFromFoot(stanceSide, offsetFromFoot);
   }


   private void computeStepLength(RobotSide swingLegSide, ReferenceFrame desiredHeadingFrame, CouplingRegistry couplingRegistry)
   {
      double swingDuration = couplingRegistry.getSingleSupportDuration();
      double stanceDuration = couplingRegistry.getDoubleSupportDuration();
      double totalDuration = swingDuration + stanceDuration;

//    FrameVector2d velocityError = desiredVelocityControlModule.getVelocityErrorInFrame(desiredHeadingFrame);
//    FrameVector2d normalizedVelocityError = new FrameVector2d(velocityError);
//    normalizedVelocityError.scale(desiredVelocityControlModule.getDesiredVelocity().length());
//
//    kpStepLength.set(normalizedVelocityError.length());

      stepLengthForDesiredVelocity.set(totalDuration * desiredVelocityControlModule.getDesiredVelocity().getX());

      double stepLengthError = stepLengthForDesiredVelocity.getDoubleValue() - stepLength.getDoubleValue();

      stepLength.set(stepLength.getDoubleValue() + stepLengthError * kpStepLength.getDoubleValue());
   }

   private void computeStepWidth(RobotSide swingLegSide, ReferenceFrame desiredHeadingFrame, CouplingRegistry couplingRegistry)
   {
      double tempStepWidth = stepWidthSlopeProfile * couplingRegistry.getDesiredVelocity().length() + stepWidthYInterceptProfile;
      MathTools.clipToMinMax(tempStepWidth, goodWalkingStepWidth, goodStandingStepWidth);
      stepWidth.set(swingLegSide.negateIfRightSide(tempStepWidth));

      stepWidth.set(stepWidth.getDoubleValue() + getStepWidthAdjustmentForTurning(swingLegSide, desiredHeadingFrame));
   }

   private double getStepWidthAdjustmentForTurning(RobotSide swingLegSide, ReferenceFrame desiredHeadingFrame)
   {
      // this is where we want to adjust the width depending on "inside" or "outside" foot
      FrameVector desiredHeading = new FrameVector(desiredHeadingFrame, 1.0, 0.0, 0.0);
      FrameVector finalHeading = desiredHeadingControlModule.getFinalHeadingTarget().changeFrameCopy(desiredHeadingFrame);
      FrameVector crossVector = new FrameVector(desiredHeading);
      crossVector.cross(desiredHeading, finalHeading);
      double magCrossProductFromDesiredToFinalHeading = crossVector.getZ();
      headingCross.set(magCrossProductFromDesiredToFinalHeading);

      double threshholdForStepWidthAdjustment = 0.00;

//    insideStepAdjustmentForTurning = Math.tan(magCrossProductFromDesiredToFinalHeading) * stepLength.getDoubleValue();
//    System.out.println("magCrossProductFromDesiredToFinalHeading = " + magCrossProductFromDesiredToFinalHeading);
//    System.out.println("insideStepAdjustmentForTurning = " + insideStepAdjustmentForTurning);

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

   private void computeFootstepYaw(RobotSide swingSide, ReferenceFrame desiredHeadingFrame)
   {
      // Compute the difference between the desired Heading frame and the frame of the swing foot.
      ReferenceFrame swingFootFrame = referenceFrames.getAnkleZUpReferenceFrames().get(swingSide);
      Transform3D headingToSwingTransform = swingFootFrame.getTransformToDesiredFrame(desiredHeadingFrame);
      Matrix3d headingToSwingRotation = new Matrix3d();
      headingToSwingTransform.get(headingToSwingRotation);

      // This can be considered as the step Yaw error.
      double headingToSwingRotationAroundZ = RotationFunctions.getYaw(headingToSwingRotation);

      // Make proportional control on the step Yaw for the desired foot step.
      stepYaw.set(KpStepYaw.getDoubleValue() * -headingToSwingRotationAroundZ);

   }




   private void checkIfStepIntersectCaptureRegion(FrameConvexPolygon2d captureRegion, FramePoint nextStep, RobotSide swingLegSide)
   {
      if (CHECK_STEP_INTO_CAPTURE_REGION)
      {
         if (captureRegion == null)
         {
            swingFootIsInsideCaptureRegion.set(false);
            nextStepIsInsideCaptureRegion.set(false);

            footstepPosition = nextStep;
         }

         else
         {
            FrameConvexPolygon2d nextStepFootPolygon = new FrameConvexPolygon2d(buildNextStepFootPolygon(nextStep.toFramePoint2d()));

            if (nextStepFootPolygon.intersectionWith(captureRegion) == null)
            {
               nextStepIsInsideCaptureRegion.set(false);
               projectDesiredStepLocationIntoCaptureRegion(nextStep, captureRegion);
            }
            else
               nextStepIsInsideCaptureRegion.set(true);

         }

         footstepPosition = nextStep;
      }

   }

   private void projectDesiredStepLocationIntoCaptureRegion(FramePoint nextStep, FrameConvexPolygon2d captureRegion)
   {
      System.out.println(this.getClass().getName() + ": project");
      FramePoint2d tempNextStep = new FramePoint2d(captureRegion.orthogonalProjectionCopy(nextStep.toFramePoint2d()));
      nextStep.setX(tempNextStep.getX());
      nextStep.setY(tempNextStep.getY());
   }

   private FrameConvexPolygon2d buildNextStepFootPolygon(FramePoint2d nextStep)
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

   public void setUpParametersForR2()
   {
      goodStandingStepWidth = (0.1016 + 0.09) * 2.0;
      goodWalkingStepWidth = 0.3272;    // 0.3272 =  minimum metabolic cost @ 0.12*Leg Length(1.245) + 2*half foot width (17.78)

      robotMaxVelocity = 0.60;
      robotMinVelocity = 0.10;

      computeStepWidthLinearProfile();

      insideStepAdjustmentForTurning = 0.0;
      outsideStepAdjustmentForTurning = 0.0;

      kpStepLength.set(0.5);
      KpStepYaw.set(1.0);
   }


   public void setupParametersForM2V2()
   {
      goodStandingStepWidth = 0.25;
      goodWalkingStepWidth = goodStandingStepWidth - 0.05;

      robotMaxVelocity = 0.5;
      robotMinVelocity = 0.1;

      computeStepWidthLinearProfile();

      insideStepAdjustmentForTurning = 0.0;
      outsideStepAdjustmentForTurning = 0.0;

      kpStepLength.set(0.5);
      KpStepYaw.set(0.5);
   }

   public void computeStepWidthLinearProfile()
   {
      stepWidthSlopeProfile = (goodStandingStepWidth - goodWalkingStepWidth) / (robotMinVelocity - robotMaxVelocity);
      stepWidthYInterceptProfile = goodStandingStepWidth - stepWidthSlopeProfile * robotMinVelocity;
   }

}

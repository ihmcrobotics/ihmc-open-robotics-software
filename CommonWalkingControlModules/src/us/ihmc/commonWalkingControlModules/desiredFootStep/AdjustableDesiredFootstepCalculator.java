package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.List;

import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredVelocityControlModule;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;



public class AdjustableDesiredFootstepCalculator extends AbstractAdjustableDesiredFootstepCalculator
{
   // Tunable robot-dependent parameters
   private double goodStandingStepWidth;
   private double goodWalkingStepWidth;

   private double robotMaxVelocity;
   private double robotMinVelocity;

   private double insideStepAdjustmentForTurning;
   private double outsideStepAdjustmentForTurning;

   private double stepWidthSlopeProfile;
   private double stepWidthYInterceptProfile;

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

   // Shared data
   private final CouplingRegistry couplingRegistry;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   // Control modules
   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final DesiredVelocityControlModule desiredVelocityControlModule;


   public AdjustableDesiredFootstepCalculator(SideDependentList<? extends ContactablePlaneBody> contactableBodies, CouplingRegistry couplingRegistry,
           DesiredHeadingControlModule desiredHeadingControlModule, DesiredVelocityControlModule desiredVelocityControlModule,
           YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry,
           SideDependentList<ReferenceFrame> ankleZUpFrames)
   {
      super(contactableBodies, getFramesToSaveFootstepIn(ankleZUpFrames), parentRegistry);

      this.couplingRegistry = couplingRegistry;
      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.desiredVelocityControlModule = desiredVelocityControlModule;
      this.ankleZUpFrames = ankleZUpFrames;
   }

   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();

      computeDesiredFootstepPosition(swingLegSide, desiredHeadingFrame);
      computeFootstepOrientation(swingLegSide, desiredHeadingFrame);
   }

   private void computeDesiredFootstepPosition(RobotSide swingLegSide, ReferenceFrame desiredHeadingFrame)
   {
      RobotSide supportLegSide = swingLegSide.getOppositeSide();

      // Compute previous Step Length
      computePreviousStepLength(desiredHeadingFrame);

      // Compute Step Length
      computeStepLength();

      // Compute Width
      computeStepWidth(swingLegSide, desiredHeadingFrame);

      // Create the desired footstep position using the parameters previously computed
      ReferenceFrame supportLegAnkleZUpFrame = ankleZUpFrames.get(supportLegSide);
      footstepPositions.get(swingLegSide).set(supportLegAnkleZUpFrame, stepLength.getDoubleValue(), stepWidth.getDoubleValue(), stepHeight.getDoubleValue());

//    desiredFootstepPosition.changeFrame(supportLegAnkleZUpFrame);
   }

   private void computePreviousStepLength(ReferenceFrame desiredHeadingFrame)
   {
      FramePoint leftFoot = new FramePoint(ankleZUpFrames.get(RobotSide.LEFT));
      leftFoot.changeFrame(ankleZUpFrames.get(RobotSide.RIGHT));
      FrameVector footToFoot = new FrameVector(leftFoot);
      footToFoot.changeFrame(desiredHeadingFrame);
      previousStepLength.set(Math.abs(footToFoot.getX()));
   }

   private final FrameVector2d desiredVelocity = new FrameVector2d();
   
   private void computeStepLength()
   {
      double swingDuration = couplingRegistry.getSingleSupportDuration();    // EVIL: single support duration should be computed based on desired velocity and step length, not the other way around
      double stanceDuration = couplingRegistry.getDoubleSupportDuration();
      double totalDuration = swingDuration + stanceDuration;

      // Do not multiply by 2, this will give you the stride length and not the step length !!!
      desiredVelocityControlModule.getDesiredVelocity(desiredVelocity);
      stepLengthForDesiredVelocity.set(totalDuration * desiredVelocity.getX());

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
      FrameVector2d desiredHeading = new FrameVector2d(desiredHeadingFrame, 1.0, 0.0);
      FrameVector2d finalHeading = new FrameVector2d(desiredHeadingControlModule.getFinalHeadingTarget());
      finalHeading.changeFrame(desiredHeadingFrame);

//    FrameVector2d crossVector = new FrameVector2d(desiredHeading);
      double magCrossProductFromDesiredToFinalHeading = desiredHeading.cross(finalHeading);

//    double magCrossProductFromDesiredToFinalHeading = crossVector.getZ();
      headingCross.set(magCrossProductFromDesiredToFinalHeading);

      double thresholdForStepWidthAdjustment = 0.00;

      if (magCrossProductFromDesiredToFinalHeading > thresholdForStepWidthAdjustment)
      {
         if (swingLegSide == RobotSide.LEFT)
            return insideStepAdjustmentForTurning;
         else
            return outsideStepAdjustmentForTurning;
      }
      else if (magCrossProductFromDesiredToFinalHeading < -thresholdForStepWidthAdjustment)
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
//    computeFootstepYaw(swingLegSide, desiredHeadingFrame);

      // Compute Step pitch
      computeFootstepPitch();

      // Compute Step roll
      computeFootstepRoll();

      // Create the desired footstep orientation using the parameters previously computed
//    desiredFootstepOrientation = new Orientation(desiredFootstepPosition.getReferenceFrame(), stepYaw.getDoubleValue(), stepPitch.getDoubleValue(), stepRoll.getDoubleValue());
      FrameOrientation desiredFootstepOrientation = new FrameOrientation(desiredHeadingFrame, stepYaw.getDoubleValue(), stepPitch.getDoubleValue(),
                                                       stepRoll.getDoubleValue());
      desiredFootstepOrientation.changeFrame(footstepOrientations.get(swingLegSide).getReferenceFrame());

      footstepOrientations.get(swingLegSide).set(desiredFootstepOrientation);
   }

// private void computeFootstepYaw(RobotSide swingSide, ReferenceFrame desiredHeadingFrame)
// {
//    // Compute the difference between the desired Heading frame and the frame of the adjusted footstep position (yaw should be w.r.t. this frame!).
//    ReferenceFrame footstepPositionFrame = desiredFootstepPositions.get(swingSide).getReferenceFrame();
//    Transform3D headingToFootstepPositionTransform = desiredHeadingFrame.getTransformToDesiredFrame(footstepPositionFrame);
//    Matrix3d headingToSwingRotation = new Matrix3d();
//    headingToFootstepPositionTransform.get(headingToSwingRotation);
//    stepYaw.set(RotationFunctions.getYaw(headingToSwingRotation));
// }


   private void computeFootstepPitch()
   {
   }


   private void computeFootstepRoll()
   {
   }

   public void setupParametersForR2()
   {
      goodStandingStepWidth = (0.1016 + 0.09) * 2.0;
      goodWalkingStepWidth = 0.2;    // 0.35;    // 0.3272;    // 0.3272 =  minimum metabolic cost @ 0.12*Leg Length(1.245) + 2*half foot width (17.78)

      robotMaxVelocity = 0.60;
      robotMinVelocity = 0.10;

      minimalStepLength.set(0.4);

      computeStepWidthLinearProfile();

      stepPitch.set(-0.25);

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

      stepPitch.set(-0.25);

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

   private static SideDependentList<ReferenceFrame> getFramesToSaveFootstepIn(SideDependentList<ReferenceFrame> ankleZUpFrames)
   {
      return new SideDependentList<ReferenceFrame>(ankleZUpFrames.get(RobotSide.RIGHT), ankleZUpFrames.get(RobotSide.LEFT));    // switch
   }

   protected List<FramePoint> getContactPoints(RobotSide swingSide)
   {
      return contactableBodies.get(swingSide).getContactPointsCopy();
   }
   
   public boolean isDone()
   {
      return false;
   }
}

package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import us.ihmc.commonWalkingControlModules.configurations.PelvisOffsetWhileWalkingParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class PelvisOffsetTrajectoryWhileWalking
{
   private static final double maxOrientationRate = 0.8;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean isStanding = new YoBoolean("pelvisIsStanding", registry);
   private final YoBoolean isInTransfer = new YoBoolean("pelvisInInTransfer", registry);

   private final YoBoolean addPelvisOffsetsBasedOnStep = new YoBoolean("addPelvisOffsetsBasedOnStep", registry);

   private final YoDouble previousSwingDuration = new YoDouble("pelvisPreviousSwingDuration", registry);
   private final YoDouble transferDuration = new YoDouble("pelvisTransferDuration", registry);
   private final YoDouble swingDuration = new YoDouble("pelvisSwingDuration", registry);
   private final YoDouble nextTransferDuration = new YoDouble("pelvisNextTransferDuration", registry);
   private final YoDouble nextSwingDuration = new YoDouble("pelvisNextSwingDuration", registry);

   private final YoDouble pelvisYawSineFrequency = new YoDouble("pelvisYawSineFrequency", registry);
   private final YoDouble pelvisYawSineMagnitude = new YoDouble("pelvisYawSineMagnitude", registry);
   private final YoDouble pelvisYawAngleRatio = new YoDouble("pelvisYawAngleRatio", registry);
   private final YoDouble pelvisYawStepLengthThreshold = new YoDouble("pelvisYawStepLengthThreshold", registry);

   private final YoDouble pelvisPitchAngleRatio = new YoDouble("pelvisPitchAngleRatio", registry);
   private final YoDouble fractionOfSwingPitchingFromUpcomingLeg = new YoDouble("pelvisFractionOfSwingPitchingFromUpcomingLeg", registry);
   private final YoDouble fractionOfSwingPitchingFromSwingLeg = new YoDouble("pelvisFractionOfSwingPitchingFromSwingLeg", registry);

   private final YoDouble yoTime;
   private final YoDouble timeInState = new YoDouble("pelvisOrientationTimeInState", registry);

   private final YoDouble leadingLegAngle = new YoDouble("pelvisPitchLeadingLegAngle", registry);
   private final YoDouble trailingLegAngle = new YoDouble("pelvisPitchTrailingLegAngle", registry);
   private final YoDouble interpolatedLegAngle = new YoDouble("pelvisPitchInterpolatedLegAngle", registry);

   private final YoFrameYawPitchRoll desiredWalkingPelvisOffsetOrientation = new YoFrameYawPitchRoll("desiredWalkingPelvisOffset", worldFrame, registry);
   private final RateLimitedYoFrameQuaternion limitedDesiredWalkingPelvisOffsetOrientation;

   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;

   private final ReferenceFrame nextSoleZUpFrame;
   private final ReferenceFrame nextSoleFrame;

   private final ReferenceFrame pelvisFrame;

   private RobotSide supportSide;
   private Footstep nextFootstep;

   private double initialTime;

   public PelvisOffsetTrajectoryWhileWalking(HighLevelHumanoidControllerToolbox controllerToolbox, PelvisOffsetWhileWalkingParameters pelvisOffsetWhileWalkingParameters,
         YoVariableRegistry parentRegistry)
   {
      this(controllerToolbox.getYoTime(), controllerToolbox.getReferenceFrames(), pelvisOffsetWhileWalkingParameters, controllerToolbox.getControlDT(), parentRegistry);
   }

   public PelvisOffsetTrajectoryWhileWalking(YoDouble yoTime, CommonHumanoidReferenceFrames referenceFrames,
         PelvisOffsetWhileWalkingParameters pelvisOffsetWhileWalkingParameters, double controlDT, YoVariableRegistry parentRegistry)
   {
      this(yoTime, referenceFrames.getSoleZUpFrames(), referenceFrames.getPelvisFrame(), pelvisOffsetWhileWalkingParameters, controlDT, parentRegistry);
   }

   public PelvisOffsetTrajectoryWhileWalking(YoDouble yoTime, SideDependentList<? extends ReferenceFrame> soleZUpFrames, ReferenceFrame pelvisFrame,
         PelvisOffsetWhileWalkingParameters pelvisOffsetWhileWalkingParameters, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.yoTime = yoTime;
      this.soleZUpFrames = soleZUpFrames;
      this.pelvisFrame = pelvisFrame;

      nextSoleFrame = new ReferenceFrame("nextSoleFrame", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            nextFootstep.getSoleReferenceFrame().getTransformToDesiredFrame(transformToParent, getParent());
         }
      };
      nextSoleZUpFrame = new ZUpFrame(worldFrame, nextSoleFrame, "nextAnkleZUp");

      addPelvisOffsetsBasedOnStep.set(pelvisOffsetWhileWalkingParameters.addPelvisOrientationOffsetsFromWalkingMotion());
      pelvisPitchAngleRatio.set(pelvisOffsetWhileWalkingParameters.getPelvisPitchRatioOfLegAngle());
      pelvisYawAngleRatio.set(pelvisOffsetWhileWalkingParameters.getPelvisYawRatioOfStepAngle());
      pelvisYawStepLengthThreshold.set(pelvisOffsetWhileWalkingParameters.getStepLengthToAddYawingMotion());

      fractionOfSwingPitchingFromSwingLeg.set(pelvisOffsetWhileWalkingParameters.getFractionOfSwingPitchingFromSwingLeg());
      fractionOfSwingPitchingFromUpcomingLeg.set(pelvisOffsetWhileWalkingParameters.getFractionOfSwingPitchingFromUpcomingLeg());

      YoDouble maxPelvisOrientationRate = new YoDouble("pelvisMaxOrientationRate", registry);
      maxPelvisOrientationRate.set(maxOrientationRate);
      limitedDesiredWalkingPelvisOffsetOrientation = new RateLimitedYoFrameQuaternion("desiredWalkingPelvisOffset", "Limited", registry,
            maxPelvisOrientationRate, controlDT, desiredWalkingPelvisOffsetOrientation.getReferenceFrame());

      parentRegistry.addChild(registry);
   }

   public void setUpcomingFootstep(Footstep upcomingFootstep)
   {
      nextFootstep = upcomingFootstep;
      supportSide = upcomingFootstep.getRobotSide().getOppositeSide();

      updateFrames();
   }


   public void initializeStanding()
   {
      isStanding.set(true);
      isInTransfer.set(false);

      reset();
   }

   private final FramePoint3D tmpPoint = new FramePoint3D();
   public void initializeTransfer(RobotSide transferToSide, double transferDuration, double swingDuration)
   {
      supportSide = transferToSide;
      this.transferDuration.set(transferDuration);
      this.previousSwingDuration.set(this.swingDuration.getDoubleValue());
      this.swingDuration.set(swingDuration);

      initialTime = yoTime.getDoubleValue();

      // compute pelvis transfer magnitude
      tmpPoint.setToZero(soleZUpFrames.get(transferToSide));
      tmpPoint.changeFrame(soleZUpFrames.get(transferToSide.getOppositeSide()));
      double stepAngle = computeStepAngle(tmpPoint, transferToSide);
      double pelvisYawSineMagnitude = pelvisYawAngleRatio.getDoubleValue() * stepAngle;

      // compute pelvis frequency
      double initialPelvisDesiredYaw = desiredWalkingPelvisOffsetOrientation.getYaw(); // use to stitch together from the previous yaw
      double pelvisYawSineFrequency = 0.0;
      if (pelvisYawSineMagnitude != initialPelvisDesiredYaw)
         pelvisYawSineFrequency = 1.0 / (2.0 * Math.PI * transferDuration) * Math.asin(-initialPelvisDesiredYaw / pelvisYawSineMagnitude);

      this.pelvisYawSineMagnitude.set(pelvisYawSineMagnitude);
      this.pelvisYawSineFrequency.set(pelvisYawSineFrequency);

      isStanding.set(false);
      isInTransfer.set(true);
   }

   public void initializeSwing(RobotSide supportSide, double currentSwingDuration, double nextTransferDuration, double nextSwingDuration)
   {
      this.supportSide = supportSide;
      this.swingDuration.set(currentSwingDuration);
      this.nextTransferDuration.set(nextTransferDuration);
      this.nextSwingDuration.set(nextSwingDuration);

      initialTime = yoTime.getDoubleValue();

      // compute pelvis swing magnitude
      tmpPoint.setToZero(nextSoleFrame);
      tmpPoint.changeFrame(soleZUpFrames.get(supportSide));
      double stepAngle = computeStepAngle(tmpPoint, supportSide);
      double pelvisYawSineMagnitude = pelvisYawAngleRatio.getDoubleValue() * stepAngle;

      // compute pelvis frequency
      double stepDuration = transferDuration.getDoubleValue() + currentSwingDuration;
      double pelvisYawSineFrequency = 1.0 / (2.0 * stepDuration);

      this.pelvisYawSineMagnitude.set(pelvisYawSineMagnitude);
      this.pelvisYawSineFrequency.set(pelvisYawSineFrequency);

      isStanding.set(false);
      isInTransfer.set(false);
   }

   public void update()
   {
      if (isStanding.getBooleanValue() || !addPelvisOffsetsBasedOnStep.getBooleanValue())
      {
         desiredWalkingPelvisOffsetOrientation.setToZero();
      }
      else
      {
         timeInState.set(yoTime.getDoubleValue() - initialTime);

         if (isInTransfer.getBooleanValue())
         {
            updatePelvisPitchOffsetInTransfer(supportSide);
         }
         else

         {
            updateFrames();
            updatePelvisPitchOffsetInSwing(supportSide);
         }
         updatePelvisYaw();
      }

      limitedDesiredWalkingPelvisOffsetOrientation.update(desiredWalkingPelvisOffsetOrientation);
   }

   public void addAngularOffset(FrameQuaternion orientationToPack)
   {
      orientationToPack.preMultiply(limitedDesiredWalkingPelvisOffsetOrientation);
   }

   private void updateFrames()
   {
      nextSoleFrame.update();
      nextSoleZUpFrame.update();
   }

   private void reset()
   {
      swingDuration.set(0.0);
      transferDuration.set(0.0);
      nextTransferDuration.set(0.0);
   }

   private double computeStepAngle(FramePoint3D footLocation, RobotSide supportSide)
   {
      double stepAngle = 0.0;
      if (Math.abs(footLocation.getX()) > pelvisYawStepLengthThreshold.getDoubleValue())
         stepAngle = Math.atan2(footLocation.getX(), Math.abs(footLocation.getY()));

      return supportSide.negateIfRightSide(stepAngle);
   }

   private void updatePelvisPitchOffsetInTransfer(RobotSide transferToSide)
   {
      double leadingLegAngle = computeAngleFromSoleToPelvis(soleZUpFrames.get(transferToSide));
      this.leadingLegAngle.set(leadingLegAngle);

      double trailingLegAngle = computeAngleFromSoleToPelvis(soleZUpFrames.get(transferToSide.getOppositeSide()));
      this.trailingLegAngle.set(trailingLegAngle);

      double timeSpentOnPreviousSwing = fractionOfSwingPitchingFromUpcomingLeg.getDoubleValue() * previousSwingDuration.getDoubleValue();
      double timeSpentOnNextSwing = fractionOfSwingPitchingFromSwingLeg.getDoubleValue() * swingDuration.getDoubleValue();

      double timeInInterpolation = timeInState.getDoubleValue() + timeSpentOnPreviousSwing;
      double totalInterpolationTime = timeSpentOnPreviousSwing + transferDuration.getDoubleValue() + timeSpentOnNextSwing;
      double ratioInInterpolation = timeInInterpolation / totalInterpolationTime;

      double interpolatedLegAngle = InterpolationTools.hermiteInterpolate(trailingLegAngle, leadingLegAngle, ratioInInterpolation);
      this.interpolatedLegAngle.set(interpolatedLegAngle);

      double desiredPitch = pelvisPitchAngleRatio.getDoubleValue() * interpolatedLegAngle;

      desiredWalkingPelvisOffsetOrientation.setPitch(desiredPitch);
   }

   private void updatePelvisPitchOffsetInSwing(RobotSide supportSide)
   {
      double trailingLegAngle = computeAngleFromSoleToPelvis(soleZUpFrames.get(supportSide));
      this.trailingLegAngle.set(trailingLegAngle);

      double ratioInState = timeInState.getDoubleValue() / swingDuration.getDoubleValue();

      double swingLegAngle, interpolatedLegAngle;
      if (ratioInState < fractionOfSwingPitchingFromSwingLeg.getDoubleValue())
      { // Interpolate against the swinging leg angle for the first phase of the swing cycle
         double timeSpentOnPreviousSwing = fractionOfSwingPitchingFromUpcomingLeg.getDoubleValue() * previousSwingDuration.getDoubleValue();
         double timeSpentOnCurrentSwing = fractionOfSwingPitchingFromSwingLeg.getDoubleValue() * swingDuration.getDoubleValue();

         double timeInInterpolation = timeInState.getDoubleValue() + timeSpentOnPreviousSwing + transferDuration.getDoubleValue();
         double totalInterpolationTime = timeSpentOnPreviousSwing + transferDuration.getDoubleValue() + timeSpentOnCurrentSwing;
         double ratioInInterpolation = timeInInterpolation / totalInterpolationTime;

         swingLegAngle = computeAngleFromSoleToPelvis(soleZUpFrames.get(supportSide.getOppositeSide()));

         interpolatedLegAngle = InterpolationTools.hermiteInterpolate(swingLegAngle, trailingLegAngle, ratioInInterpolation);

      }
      else if (ratioInState > (1.0 - fractionOfSwingPitchingFromUpcomingLeg.getDoubleValue()))
      {
         double timeSpentOnCurrentSwing = fractionOfSwingPitchingFromUpcomingLeg.getDoubleValue() * swingDuration.getDoubleValue();
         double timeSpentOnNextSwing = fractionOfSwingPitchingFromSwingLeg.getDoubleValue() * nextSwingDuration.getDoubleValue();

         double timeInInterpolation = timeInState.getDoubleValue() - (swingDuration.getDoubleValue() - timeSpentOnCurrentSwing);
         double totalInterpolationTime = timeSpentOnCurrentSwing + nextTransferDuration.getDoubleValue() + timeSpentOnNextSwing;
         double ratioInInterpolation = timeInInterpolation / totalInterpolationTime;

         swingLegAngle = computeAngleFromSoleToPelvis(nextSoleZUpFrame);

         interpolatedLegAngle = InterpolationTools.hermiteInterpolate(trailingLegAngle, swingLegAngle, ratioInInterpolation);
      }
      else
      {
         swingLegAngle = 0.0;
         interpolatedLegAngle = trailingLegAngle;
      }

      double desiredPitch = pelvisPitchAngleRatio.getDoubleValue() * interpolatedLegAngle;
      this.interpolatedLegAngle.set(interpolatedLegAngle);
      this.leadingLegAngle.set(swingLegAngle);

      desiredWalkingPelvisOffsetOrientation.setPitch(desiredPitch);
   }

   private final FramePoint3D tempPoint = new FramePoint3D();
   private double computeAngleFromSoleToPelvis(ReferenceFrame soleFrame)
   {
      tempPoint.setToZero(pelvisFrame);
      tempPoint.changeFrame(soleFrame);

      double distanceFromSupportFoot = tempPoint.getX();
      double heightFromSupportFoot = tempPoint.getZ();
      return Math.atan2(distanceFromSupportFoot, heightFromSupportFoot);
   }


   private void updatePelvisYaw()
   {
      double timeInSine = timeInState.getDoubleValue();
      if (isInTransfer.getBooleanValue())
         timeInSine -= transferDuration.getDoubleValue();

      double radiansPerSecond = pelvisYawSineFrequency.getDoubleValue() * Math.PI * 2.0;
      double desiredYaw = pelvisYawSineMagnitude.getDoubleValue() * Math.sin(timeInSine * radiansPerSecond);

      desiredWalkingPelvisOffsetOrientation.setYaw(desiredYaw);
   }
}

package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PelvisOffsetTrajectoryWhileWalking
{
   private static final double pelvisPitchPercentToStopPitching = 0.8;
   private static final double offsetPhaseInDuration = 0.01;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable isStanding = new BooleanYoVariable("pelvisIsStanding", registry);
   private final BooleanYoVariable isInTransfer = new BooleanYoVariable("pelvisInInTransfer", registry);

   private final BooleanYoVariable addPelvisOffsetsBasedOnStep = new BooleanYoVariable("addPelvisOffsetsBasedOnStep", registry);

   private final DoubleYoVariable transferDuration = new DoubleYoVariable("pelvisTransferDuration", registry);
   private final DoubleYoVariable swingDuration = new DoubleYoVariable("pelvisSwingDuration", registry);
   private final DoubleYoVariable nextTransferDuration = new DoubleYoVariable("pelvisNextTransferDuration", registry);

   private final DoubleYoVariable pelvisYawSineFrequency = new DoubleYoVariable("pelvisYawSineFrequency", registry);
   private final DoubleYoVariable pelvisYawSineMagnitude = new DoubleYoVariable("pelvisYawSineMagnitude", registry);
   private final DoubleYoVariable pelvisYawAngleRatio = new DoubleYoVariable("pelvisYawAngleRatio", registry);
   private final DoubleYoVariable pelvisYawStepLengthThreshold = new DoubleYoVariable("pelvisYawStepLengthThreshold", registry);

   private final DoubleYoVariable pelvisPitchAngleRatio = new DoubleYoVariable("pelvisPitchAngleRatio", registry);
   private final DoubleYoVariable pelvisPitchPercentSwingToStopPitching = new DoubleYoVariable("pelvisPitchPercentSwingToStopPitching", registry);

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable timeInState = new DoubleYoVariable("pelvisOrientationTimeInState", registry);
   private final DoubleYoVariable phaseInDuration = new DoubleYoVariable("pelvisOrientationPhaseInDuration", registry);

   private final DoubleYoVariable leadingLegAngle = new DoubleYoVariable("pelvisPitchLeadingLegAngle", registry);
   private final DoubleYoVariable trailingLegAngle = new DoubleYoVariable("pelvisPitchTrailingLegAngle", registry);
   private final DoubleYoVariable interpolatedLegAngle = new DoubleYoVariable("pelvisPitchInterpolatedLegAngle", registry);

   private final YoFrameOrientation desiredWalkingPelvisOffsetOrientation = new YoFrameOrientation("desiredWalkingPelvisOffset", worldFrame, registry);

   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   private final ReferenceFrame nextAnkleZUpFrame;
   private final ReferenceFrame nextAnkleFrame;
   private final ReferenceFrame nextSoleFrame;

   private final ReferenceFrame pelvisFrame;

   private RobotSide supportSide;
   private Footstep nextFootstep;

   private double initialTime;

   private double initialPitchOffset;
   private double initialYawOffset;

   public PelvisOffsetTrajectoryWhileWalking(DoubleYoVariable yoTime, SideDependentList<RigidBodyTransform> transformsFromAnkleToSole,
         SideDependentList<ReferenceFrame> ankleZUpFrames, ReferenceFrame pelvisFrame, WalkingControllerParameters walkingControllerParameters,
         YoVariableRegistry parentRegistry)
   {
      this.yoTime = yoTime;
      this.ankleZUpFrames = ankleZUpFrames;
      this.pelvisFrame = pelvisFrame;

      nextSoleFrame = new ReferenceFrame("nextSoleFrame", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            nextFootstep.getSoleReferenceFrame().getTransformToDesiredFrame(transformToParent, parentFrame);
         }
      };
      nextAnkleFrame = new ReferenceFrame("ankleZUpFrame", nextSoleFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            RigidBodyTransform ankleToSole = transformsFromAnkleToSole.get(nextFootstep.getRobotSide());
            transformToParent.set(ankleToSole);
         }
      };
      nextAnkleZUpFrame = new ZUpFrame(worldFrame, nextAnkleFrame, "nextAnkleZUp");

      addPelvisOffsetsBasedOnStep.set(walkingControllerParameters.addPelvisOrientationOffsetsFromWalkingMotion());
      pelvisPitchAngleRatio.set(walkingControllerParameters.pelvisPitchRatioOfLegAngle());
      pelvisPitchPercentSwingToStopPitching.set(pelvisPitchPercentToStopPitching);
      pelvisYawAngleRatio.set(walkingControllerParameters.pelvisYawRatioOfStepAngle());
      phaseInDuration.set(offsetPhaseInDuration);
      pelvisYawStepLengthThreshold.set(walkingControllerParameters.stepLengthToAddYawingMotion());

      parentRegistry.addChild(registry);
   }

   public void setUpcomingFootstep(Footstep upcomingFootstep)
   {
      nextFootstep = upcomingFootstep;
      supportSide = upcomingFootstep.getRobotSide().getOppositeSide();

      updateFrames();
   }

   private void updateFrames()
   {
      nextSoleFrame.update();
      nextAnkleFrame.update();
      nextAnkleZUpFrame.update();
   }


   private double computeStepAngle(FramePoint footLocation, RobotSide supportSide)
   {
      double stepAngle = 0.0;
      if (Math.abs(footLocation.getX()) > pelvisYawStepLengthThreshold.getDoubleValue())
         stepAngle = Math.atan2(footLocation.getX(), Math.abs(footLocation.getY()));

      return supportSide.negateIfRightSide(stepAngle);
   }

   public void initializeStanding()
   {
      isStanding.set(true);
      swingDuration.set(0.0);
      isInTransfer.set(false);
   }

   private final FramePoint tmpPoint = new FramePoint();
   public void initializeTransfer(RobotSide transferToSide, double transferDuration)
   {
      supportSide = transferToSide;
      this.transferDuration.set(transferDuration);

      initialTime = yoTime.getDoubleValue();

      // compute pelvis transfer magnitude
      tmpPoint.setToZero(ankleZUpFrames.get(transferToSide));
      tmpPoint.changeFrame(ankleZUpFrames.get(transferToSide.getOppositeSide()));
      double stepAngle = computeStepAngle(tmpPoint, transferToSide);
      double pelvisYawSineMagnitude = pelvisYawAngleRatio.getDoubleValue() * stepAngle;

      // compute pelvis frequency
      double initialPelvisDesiredYaw = desiredWalkingPelvisOffsetOrientation.getYaw().getDoubleValue(); // use to stitch together from the previous yaw
      double pelvisYawSineFrequency = 0.0;
      if (pelvisYawSineMagnitude != initialPelvisDesiredYaw)
         pelvisYawSineFrequency = 1.0 / (2.0 * Math.PI * transferDuration) * Math.asin(-initialPelvisDesiredYaw / pelvisYawSineMagnitude);

      this.pelvisYawSineMagnitude.set(pelvisYawSineMagnitude);
      this.pelvisYawSineFrequency.set(pelvisYawSineFrequency);

      initialPitchOffset = desiredWalkingPelvisOffsetOrientation.getPitch().getDoubleValue();
      initialYawOffset = desiredWalkingPelvisOffsetOrientation.getYaw().getDoubleValue();

      isStanding.set(false);
      isInTransfer.set(true);
   }

   public void initializeSwing(RobotSide supportSide, double swingDuration, double nextTransferDuration)
   {
      this.supportSide = supportSide;
      this.swingDuration.set(swingDuration);
      this.nextTransferDuration.set(nextTransferDuration);

      initialTime = yoTime.getDoubleValue();

      // compute pelvis swing magnitude
      tmpPoint.setToZero(nextAnkleFrame);
      tmpPoint.changeFrame(ankleZUpFrames.get(supportSide));
      double stepAngle = computeStepAngle(tmpPoint, supportSide);
      double pelvisYawSineMagnitude = pelvisYawAngleRatio.getDoubleValue() * stepAngle;

      // compute pelvis frequency
      double stepDuration = transferDuration.getDoubleValue() + swingDuration;
      double pelvisYawSineFrequency = 1.0 / (2.0 * stepDuration);

      this.pelvisYawSineMagnitude.set(pelvisYawSineMagnitude);
      this.pelvisYawSineFrequency.set(pelvisYawSineFrequency);

      initialPitchOffset = desiredWalkingPelvisOffsetOrientation.getPitch().getDoubleValue();
      initialYawOffset = desiredWalkingPelvisOffsetOrientation.getYaw().getDoubleValue();
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
   }

   public void getAngularData(FrameOrientation orientationToPack)
   {
      double desiredYaw = orientationToPack.getYaw();
      double desiredPitch = orientationToPack.getPitch();
      double desiredRoll = orientationToPack.getRoll();

      desiredYaw += desiredWalkingPelvisOffsetOrientation.getYaw().getDoubleValue();
      desiredPitch += desiredWalkingPelvisOffsetOrientation.getPitch().getDoubleValue();
      desiredRoll += desiredWalkingPelvisOffsetOrientation.getRoll().getDoubleValue();

      orientationToPack.setYawPitchRoll(desiredYaw, desiredPitch, desiredRoll);
   }

   // // TODO: 4/25/17  make the angle also a function of the height

   private void updatePelvisPitchOffsetInTransfer(RobotSide transferToSide)
   {
      double leadingLegAngle = computeAngleFromAnkleToPelvis(ankleZUpFrames.get(transferToSide));
      this.leadingLegAngle.set(leadingLegAngle);

      double trailingLegAngle = computeAngleFromAnkleToPelvis(ankleZUpFrames.get(transferToSide.getOppositeSide()));
      this.trailingLegAngle.set(trailingLegAngle);

      double swingForInterpolation = (1.0 - pelvisPitchPercentSwingToStopPitching.getDoubleValue()) * swingDuration.getDoubleValue();
      double timeInInterpolation = timeInState.getDoubleValue() + swingForInterpolation;
      double percentInInterpolation = timeInInterpolation / (swingForInterpolation + transferDuration.getDoubleValue());

      double interpolatedLegAngle = InterpolationTools.hermiteInterpolate(trailingLegAngle, leadingLegAngle, percentInInterpolation);
      this.interpolatedLegAngle.set(interpolatedLegAngle);

      double desiredPitch = pelvisPitchAngleRatio.getDoubleValue() * interpolatedLegAngle;

      // take care of the phase in period to avoid any discontinuities
      desiredPitch = computePhaseInBlending(initialPitchOffset, desiredPitch);

      desiredWalkingPelvisOffsetOrientation.setPitch(desiredPitch);
   }

   private void updatePelvisPitchOffsetInSwing(RobotSide supportSide)
   {
      double trailingLegAngle = computeAngleFromAnkleToPelvis(ankleZUpFrames.get(supportSide));
      this.trailingLegAngle.set(trailingLegAngle);

      double percentInState = timeInState.getDoubleValue() / swingDuration.getDoubleValue();
      double leadingLegAngle = computeAngleFromAnkleToPelvis(nextAnkleZUpFrame);
      this.leadingLegAngle.set(leadingLegAngle);

      double desiredPitch;
      if (percentInState > pelvisPitchPercentSwingToStopPitching.getDoubleValue())
      {

         double swingForInterpolation = (1.0 - pelvisPitchPercentSwingToStopPitching.getDoubleValue()) * swingDuration.getDoubleValue();
         double timeInInterpolation = timeInState.getDoubleValue() - (swingDuration.getDoubleValue() - swingForInterpolation);
         double percentInInterpolation = timeInInterpolation / (swingForInterpolation + nextTransferDuration.getDoubleValue());

         double interpolatedLegAngle = InterpolationTools.hermiteInterpolate(trailingLegAngle, leadingLegAngle, percentInInterpolation);
         this.interpolatedLegAngle.set(interpolatedLegAngle);

         desiredPitch = pelvisPitchAngleRatio.getDoubleValue() * interpolatedLegAngle;
      }
      else
      {
         this.interpolatedLegAngle.set(trailingLegAngle);
         desiredPitch = pelvisPitchAngleRatio.getDoubleValue() * trailingLegAngle;
      }

      // take care of the phase in period to avoid any discontinuities
      desiredPitch = computePhaseInBlending(initialPitchOffset, desiredPitch);

      desiredWalkingPelvisOffsetOrientation.setPitch(desiredPitch);
   }

   private final FramePoint tempPoint = new FramePoint();
   private double computeAngleFromAnkleToPelvis(ReferenceFrame ankleFrame)
   {
      tempPoint.setToZero(pelvisFrame);
      tempPoint.changeFrame(ankleFrame);

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

      // take care of the phase in period to avoid any discontinuities
      desiredYaw = computePhaseInBlending(initialYawOffset, desiredYaw);

      desiredWalkingPelvisOffsetOrientation.setYaw(desiredYaw);
   }

   private double computePhaseInBlending(double initialValue, double desiredValue)
   {
      double phaseInAlpha = MathTools.clamp(timeInState.getDoubleValue() / phaseInDuration.getDoubleValue(), 0.0, 1.0);
      return InterpolationTools.linearInterpolate(initialValue, desiredValue, phaseInAlpha);
   }
}

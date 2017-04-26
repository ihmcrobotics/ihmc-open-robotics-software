package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PelvisOffsetTrajectoryWhileWalking
{
   private static final double pelvisYawRatio = 0.3;
   private static final double pelvisPitchRatio = 0.3;
   private static final double pelvisPitchPercentToStopPitching = 0.8;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable isStanding = new BooleanYoVariable("pelvisIsStanding", registry);
   private final BooleanYoVariable isInTransfer = new BooleanYoVariable("pelvisInInTransfer", registry);

   private final DoubleYoVariable transferDuration = new DoubleYoVariable("pelvisTransferDuration", registry);
   private final DoubleYoVariable swingDuration = new DoubleYoVariable("pelvisSwingDuration", registry);
   private final DoubleYoVariable nextTransferDuration = new DoubleYoVariable("pelvisNextTransferDuration", registry);

   private final DoubleYoVariable pelvisYawSineFrequency = new DoubleYoVariable("pelvisYawSineFrequency", registry);
   private final DoubleYoVariable pelvisYawSineMagnitude = new DoubleYoVariable("pelvisYawSineMagnitude", registry);
   private final DoubleYoVariable pelvisYawAngleRatio = new DoubleYoVariable("pelvisYawAngleRatio", registry);

   private final BooleanYoVariable addPelvisOffsetsBasedOnStep = new BooleanYoVariable("addPelvisOffsetsBasedOnStep", registry);
   private final DoubleYoVariable pelvisPitchAngleRatio = new DoubleYoVariable("pelvisPitchAngleRatio", registry);
   private final DoubleYoVariable pelvisPitchPercentSwingToStopPitching = new DoubleYoVariable("pelvisPitchPercentSwingToStopPitching", registry);

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable timeInState = new DoubleYoVariable("pelvisOrientationTimeInState", registry);

   private final SideDependentList<RigidBodyTransform> transformsFromAnkleToSole;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   private final FramePoint upcomingFootstepLocation = new FramePoint();
   private final FrameOrientation upcomingFootstepOrientation = new FrameOrientation();

   private final FrameOrientation desiredPelvisOffsetWhileWalking = new FrameOrientation(worldFrame);

   private final ReferenceFrame nextAnkleZUpFrame;
   private final ReferenceFrame nextAnkleFrame;
   private final ReferenceFrame nextSoleFrame;

   private final ReferenceFrame pelvisFrame;

   private RobotSide supportSide;
   private Footstep nextFootstep;

   private double initialTime;


   public PelvisOffsetTrajectoryWhileWalking(DoubleYoVariable yoTime, SideDependentList<RigidBodyTransform> transformsFromAnkleToSole,
         SideDependentList<ReferenceFrame> ankleZUpFrames, ReferenceFrame pelvisFrame, YoVariableRegistry parentRegistry)
   {
      this.yoTime = yoTime;
      this.transformsFromAnkleToSole = transformsFromAnkleToSole;
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
      parentRegistry.addChild(registry);
   }

   public void setUpcomingFootstep(Footstep upcomingFootstep)
   {
      nextFootstep = upcomingFootstep;
      supportSide = upcomingFootstep.getRobotSide().getOppositeSide();

      nextSoleFrame.update();
      nextAnkleFrame.update();
      nextAnkleZUpFrame.update();

      RigidBodyTransform ankleToSole = transformsFromAnkleToSole.get(upcomingFootstep.getRobotSide());
      upcomingFootstep.getAnkleOrientation(upcomingFootstepOrientation, ankleToSole);
      upcomingFootstep.getAnklePosition(upcomingFootstepLocation, ankleToSole);
   }

   private double computeStepAngle(FramePoint footLocation, RobotSide supportSide)
   {
      double stepAngle = 0.0;
      if (Math.abs(footLocation.getX()) > 0.03)
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
      double initialPelvisDesiredYaw = desiredPelvisOffsetWhileWalking.getYaw(); // use to stitch together from the previous yaw
      double pelvisYawSineFrequency = 0.0;
      if (pelvisYawSineMagnitude != initialPelvisDesiredYaw)
         pelvisYawSineFrequency = 1.0 / (2.0 * Math.PI * transferDuration) * Math.asin(-initialPelvisDesiredYaw / pelvisYawSineMagnitude);

      this.pelvisYawSineMagnitude.set(pelvisYawSineMagnitude);
      this.pelvisYawSineFrequency.set(pelvisYawSineFrequency);

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
      tmpPoint.setIncludingFrame(upcomingFootstepLocation);
      tmpPoint.changeFrame(ankleZUpFrames.get(supportSide));
      double stepAngle = computeStepAngle(tmpPoint, supportSide);
      double pelvisYawSineMagnitude = pelvisYawAngleRatio.getDoubleValue() * stepAngle;

      // compute pelvis frequency
      double stepDuration = transferDuration.getDoubleValue() + swingDuration;

      this.pelvisYawSineMagnitude.set(pelvisYawSineMagnitude);
      this.pelvisYawSineFrequency.set(1.0 / (2.0 * stepDuration));

      isStanding.set(false);
      isInTransfer.set(false);
   }

   private void computePelvisOffsetWhileWalking()
   {
      if (isStanding.getBooleanValue())
      {
         desiredPelvisOffsetWhileWalking.setToZero();
      }
      else
      {
         timeInState.set(yoTime.getDoubleValue() - initialTime);

         if (isInTransfer.getBooleanValue())
         {
            updatePelvisPitchOffsetInTransfer(supportSide);
            updatePelvisYawOffsetInTransfer();
         }
         else
         {
            updatePelvisPitchOffsetInSwing(supportSide);
            updatePelvisYawOffsetInSwing();
         }
      }
   }

   // // TODO: 4/25/17  make the angle also a function of the height
   private final FramePoint stanceLine = new FramePoint();
   private final FramePoint otherStanceLine = new FramePoint();

   private void updatePelvisPitchOffsetInTransfer(RobotSide transferToSide)
   {
      stanceLine.setToZero(pelvisFrame);
      stanceLine.changeFrame(ankleZUpFrames.get(transferToSide));
      double distanceFromSupportFoot = stanceLine.getX();
      double heightFromSupportFoot = stanceLine.getZ();
      double leadingLegAngle = Math.atan2(distanceFromSupportFoot, heightFromSupportFoot);

      otherStanceLine.setToZero(pelvisFrame);
      otherStanceLine.changeFrame(ankleZUpFrames.get(transferToSide.getOppositeSide()));
      double distanceFromTrailingFoot = otherStanceLine.getX();
      double heightFromTrailingFoot = otherStanceLine.getZ();
      double trailingLegAngle = Math.atan2(distanceFromTrailingFoot, heightFromTrailingFoot);

      double swingForInterpolation = (1.0 - pelvisPitchPercentSwingToStopPitching.getDoubleValue()) * swingDuration.getDoubleValue();
      double timeInInterpolation = timeInState.getDoubleValue() + swingForInterpolation;
      double percentInInterpolation = timeInInterpolation / (swingForInterpolation + transferDuration.getDoubleValue());

      double interpolatedLegAngle = InterpolationTools.hermiteInterpolate(trailingLegAngle, leadingLegAngle, percentInInterpolation);

      double currentYaw = desiredPelvisOffsetWhileWalking.getYaw();
      double currentRoll = desiredPelvisOffsetWhileWalking.getRoll();
      double desiredPitch = pelvisPitchAngleRatio.getDoubleValue() * interpolatedLegAngle;
      desiredPelvisOffsetWhileWalking.setYawPitchRoll(currentYaw, desiredPitch, currentRoll);
   }

   private void updatePelvisPitchOffsetInSwing(RobotSide supportSide)
   {
      stanceLine.setToZero(pelvisFrame);
      stanceLine.changeFrame(ankleZUpFrames.get(supportSide));
      double distanceFromSupportFoot = stanceLine.getX();
      double heightFromSupportFoot = stanceLine.getZ();
      double supportLegAngle = Math.atan2(distanceFromSupportFoot, heightFromSupportFoot);

      double percentInState = timeInState.getDoubleValue() / swingDuration.getDoubleValue();

      if (percentInState > pelvisPitchPercentSwingToStopPitching.getDoubleValue())
      {
         otherStanceLine.setToZero(pelvisFrame);
         nextSoleFrame.update();
         nextAnkleFrame.update();
         nextAnkleZUpFrame.update();
         otherStanceLine.changeFrame(nextAnkleZUpFrame);
         double distanceFromFootstep = otherStanceLine.getX();
         double heightFromFootstep = otherStanceLine.getZ();
         double upcomingStepAngle = Math.atan2(distanceFromFootstep, heightFromFootstep);

         double swingForInterpolation = (1.0 - pelvisPitchPercentSwingToStopPitching.getDoubleValue()) * swingDuration.getDoubleValue();
         double timeInInterpolation = timeInState.getDoubleValue() - (swingDuration.getDoubleValue() - swingForInterpolation);
         double percentInInterpolation = timeInInterpolation / (swingForInterpolation + nextTransferDuration.getDoubleValue());

         double interpolatedLegAngle = InterpolationTools.hermiteInterpolate(supportLegAngle, upcomingStepAngle, percentInInterpolation);

         double currentYaw = desiredPelvisOffsetWhileWalking.getYaw();
         double currentRoll = desiredPelvisOffsetWhileWalking.getRoll();
         double desiredPitch = pelvisPitchAngleRatio.getDoubleValue() * interpolatedLegAngle;
         desiredPelvisOffsetWhileWalking.setYawPitchRoll(currentYaw, desiredPitch, currentRoll);
      }
      else
      {
         double currentYaw = desiredPelvisOffsetWhileWalking.getYaw();
         double currentRoll = desiredPelvisOffsetWhileWalking.getRoll();
         double desiredPitch = pelvisPitchAngleRatio.getDoubleValue() * supportLegAngle;
         desiredPelvisOffsetWhileWalking.setYawPitchRoll(currentYaw, desiredPitch, currentRoll);
      }

   }

   private void updatePelvisYawOffsetInTransfer()
   {
      double timeInSine = timeInState.getDoubleValue() - transferDuration.getDoubleValue();
      double yaw = pelvisYawSineMagnitude.getDoubleValue() * Math.sin(timeInSine * pelvisYawSineFrequency.getDoubleValue() * 2.0 * Math.PI);
      double currentPitch = desiredPelvisOffsetWhileWalking.getPitch();
      double currentRoll = desiredPelvisOffsetWhileWalking.getRoll();
      desiredPelvisOffsetWhileWalking.setYawPitchRoll(yaw, currentPitch, currentRoll);
   }

   private void updatePelvisYawOffsetInSwing()
   {
      double timeInStep = timeInState.getDoubleValue();
      double yaw = pelvisYawSineMagnitude.getDoubleValue() * Math.sin(timeInStep * pelvisYawSineFrequency.getDoubleValue() * 2.0 * Math.PI);
      double currentPitch = desiredPelvisOffsetWhileWalking.getPitch();
      double currentRoll = desiredPelvisOffsetWhileWalking.getRoll();
      desiredPelvisOffsetWhileWalking.setYawPitchRoll(yaw, currentPitch, currentRoll);
   }
}

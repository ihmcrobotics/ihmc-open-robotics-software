package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import us.ihmc.commonWalkingControlModules.configurations.PelvisOffsetWhileWalkingParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class PelvisOffsetTrajectoryWhileWalking
{
   private static final double maxOrientationRate = 0.8;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable isStanding = new BooleanYoVariable("pelvisIsStanding", registry);
   private final BooleanYoVariable isInTransfer = new BooleanYoVariable("pelvisInInTransfer", registry);

   private final BooleanYoVariable addPelvisOffsetsBasedOnStep = new BooleanYoVariable("addPelvisOffsetsBasedOnStep", registry);

   private final DoubleYoVariable previousSwingDuration = new DoubleYoVariable("pelvisPreviousSwingDuration", registry);
   private final DoubleYoVariable transferDuration = new DoubleYoVariable("pelvisTransferDuration", registry);
   private final DoubleYoVariable swingDuration = new DoubleYoVariable("pelvisSwingDuration", registry);
   private final DoubleYoVariable nextTransferDuration = new DoubleYoVariable("pelvisNextTransferDuration", registry);
   private final DoubleYoVariable nextSwingDuration = new DoubleYoVariable("pelvisNextSwingDuration", registry);

   private final DoubleYoVariable pelvisYawSineFrequency = new DoubleYoVariable("pelvisYawSineFrequency", registry);
   private final DoubleYoVariable pelvisYawSineMagnitude = new DoubleYoVariable("pelvisYawSineMagnitude", registry);
   private final DoubleYoVariable pelvisYawAngleRatio = new DoubleYoVariable("pelvisYawAngleRatio", registry);
   private final DoubleYoVariable pelvisYawStepLengthThreshold = new DoubleYoVariable("pelvisYawStepLengthThreshold", registry);

   private final DoubleYoVariable pelvisPitchAngleRatio = new DoubleYoVariable("pelvisPitchAngleRatio", registry);
   private final DoubleYoVariable fractionOfSwingPitchingFromUpcomingLeg = new DoubleYoVariable("pelvisFractionOfSwingPitchingFromUpcomingLeg", registry);
   private final DoubleYoVariable fractionOfSwingPitchingFromSwingLeg = new DoubleYoVariable("pelvisFractionOfSwingPitchingFromSwingLeg", registry);

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable timeInState = new DoubleYoVariable("pelvisOrientationTimeInState", registry);

   private final DoubleYoVariable leadingLegAngle = new DoubleYoVariable("pelvisPitchLeadingLegAngle", registry);
   private final DoubleYoVariable trailingLegAngle = new DoubleYoVariable("pelvisPitchTrailingLegAngle", registry);
   private final DoubleYoVariable interpolatedLegAngle = new DoubleYoVariable("pelvisPitchInterpolatedLegAngle", registry);

   private final YoFrameOrientation desiredWalkingPelvisOffsetOrientation = new YoFrameOrientation("desiredWalkingPelvisOffset", worldFrame, registry);
   private final RateLimitedYoFrameOrientation limitedDesiredWalkingPelvisOffsetOrientation;

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

   public PelvisOffsetTrajectoryWhileWalking(DoubleYoVariable yoTime, CommonHumanoidReferenceFrames referenceFrames,
         PelvisOffsetWhileWalkingParameters pelvisOffsetWhileWalkingParameters, double controlDT, YoVariableRegistry parentRegistry)
   {
      this(yoTime, referenceFrames.getSoleZUpFrames(), referenceFrames.getPelvisFrame(), pelvisOffsetWhileWalkingParameters, controlDT, parentRegistry);
   }

   public PelvisOffsetTrajectoryWhileWalking(DoubleYoVariable yoTime, SideDependentList<? extends ReferenceFrame> soleZUpFrames, ReferenceFrame pelvisFrame,
         PelvisOffsetWhileWalkingParameters pelvisOffsetWhileWalkingParameters, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.yoTime = yoTime;
      this.soleZUpFrames = soleZUpFrames;
      this.pelvisFrame = pelvisFrame;

      nextSoleFrame = new ReferenceFrame("nextSoleFrame", worldFrame)
      {
         private static final long serialVersionUID = 8082360349132384241L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            nextFootstep.getSoleReferenceFrame().getTransformToDesiredFrame(transformToParent, parentFrame);
         }
      };
      nextSoleZUpFrame = new ZUpFrame(worldFrame, nextSoleFrame, "nextAnkleZUp");

      addPelvisOffsetsBasedOnStep.set(pelvisOffsetWhileWalkingParameters.addPelvisOrientationOffsetsFromWalkingMotion());
      pelvisPitchAngleRatio.set(pelvisOffsetWhileWalkingParameters.getPelvisPitchRatioOfLegAngle());
      pelvisYawAngleRatio.set(pelvisOffsetWhileWalkingParameters.getPelvisYawRatioOfStepAngle());
      pelvisYawStepLengthThreshold.set(pelvisOffsetWhileWalkingParameters.getStepLengthToAddYawingMotion());

      fractionOfSwingPitchingFromSwingLeg.set(pelvisOffsetWhileWalkingParameters.getFractionOfSwingPitchingFromSwingLeg());
      fractionOfSwingPitchingFromUpcomingLeg.set(pelvisOffsetWhileWalkingParameters.getFractionOfSwingPitchingFromUpcomingLeg());

      DoubleYoVariable maxPelvisOrientationRate = new DoubleYoVariable("pelvisMaxOrientationRate", registry);
      maxPelvisOrientationRate.set(maxOrientationRate);
      limitedDesiredWalkingPelvisOffsetOrientation = new RateLimitedYoFrameOrientation("desiredWalkingPelvisOffset", "Limited", registry,
            maxPelvisOrientationRate, controlDT, desiredWalkingPelvisOffsetOrientation);

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

   private final FramePoint tmpPoint = new FramePoint();
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
      double initialPelvisDesiredYaw = desiredWalkingPelvisOffsetOrientation.getYaw().getDoubleValue(); // use to stitch together from the previous yaw
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

      limitedDesiredWalkingPelvisOffsetOrientation.update();
   }

   public void addAngularOffset(FrameOrientation orientationToPack)
   {
      orientationToPack.preMultiply(limitedDesiredWalkingPelvisOffsetOrientation.getFrameOrientation());
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

   private double computeStepAngle(FramePoint footLocation, RobotSide supportSide)
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

   private final FramePoint tempPoint = new FramePoint();
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

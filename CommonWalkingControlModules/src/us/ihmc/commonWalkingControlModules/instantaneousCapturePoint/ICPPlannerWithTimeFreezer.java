package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class ICPPlannerWithTimeFreezer extends ICPPlanner
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final BooleanYoVariable doTimeFreezing;
   private final BooleanYoVariable isTimeBeingFrozen;
   private final DoubleYoVariable timeDelay;
   private final DoubleYoVariable capturePointPositionError;
   private final DoubleYoVariable distanceToFreezeLine;
   private final DoubleYoVariable previousTime;
   private final DoubleYoVariable freezeTimeFactor;
   private final DoubleYoVariable maxCapturePointErrorAllowedToBeginSwingPhase;
   private final DoubleYoVariable maxAllowedCapturePointErrorWithoutPartialTimeFreeze;
   
   private final FramePoint tmpCapturePointPosition;
   private final FrameVector tmpCapturePointVelocity;
   
   public ICPPlannerWithTimeFreezer(BipedSupportPolygons bipedSupportPolygons, CommonHumanoidReferenceFrames referenceFrames,
         CapturePointPlannerParameters capturePointPlannerParameters, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(bipedSupportPolygons, referenceFrames, capturePointPlannerParameters, parentRegistry, yoGraphicsListRegistry);

      this.timeDelay = new DoubleYoVariable("icpPlannerTimeDelayFromeFreezer", registry);
      this.capturePointPositionError = new DoubleYoVariable("icpPlannerCapturePointPositionError", registry);
      this.distanceToFreezeLine = new DoubleYoVariable("icpPlannerDistanceToFreezeLine", registry);
      this.freezeTimeFactor = new DoubleYoVariable("icpPlannerFreezeTimeFactor", registry);
      this.maxCapturePointErrorAllowedToBeginSwingPhase = new DoubleYoVariable("icpPlannerMaxCapturePointErrorAllowedToBeginSwingPhase", registry);
      this.maxAllowedCapturePointErrorWithoutPartialTimeFreeze = new DoubleYoVariable("icpPlannerMaxAllowedCapturePointErrorWithoutTimeFreeze", registry);
      this.previousTime = new DoubleYoVariable("icpPlannerPreviousTime", registry);
      this.doTimeFreezing = new BooleanYoVariable("icpPlannerDoTimeFreezing", registry);
      this.isTimeBeingFrozen = new BooleanYoVariable("icpPlannerIsTimeBeingFrozen", registry);
      this.tmpCapturePointPosition = new FramePoint(worldFrame);
      this.tmpCapturePointVelocity = new FrameVector(worldFrame);
      
      this.isTimeBeingFrozen.set(false);
      this.timeDelay.set(0.0);
      this.capturePointPositionError.set(0.0);
      this.distanceToFreezeLine.set(0.0);
      this.doTimeFreezing.set(capturePointPlannerParameters.getDoTimeFreezing());

      this.maxCapturePointErrorAllowedToBeginSwingPhase.set(capturePointPlannerParameters.getMaxInstantaneousCapturePointErrorForStartingSwing());
      this.maxAllowedCapturePointErrorWithoutPartialTimeFreeze.set(capturePointPlannerParameters.getMaxAllowedErrorWithoutPartialTimeFreeze());
      this.freezeTimeFactor.set(capturePointPlannerParameters.getFreezeTimeFactor());
   }

   public void packDesiredCapturePointPositionAndVelocity(FramePoint desiredCapturePointPositionToPack, FrameVector desiredCapturePointVelocityToPack, FramePoint currentCapturePointPosition,
         double time)
   {
      super.packDesiredCapturePointPositionAndVelocity(desiredCapturePointPositionToPack, desiredCapturePointVelocityToPack, time);

      if (doTimeFreezing.getBooleanValue())
      {
         tmpCapturePointPosition.setIncludingFrame(desiredCapturePointPositionToPack);
         tmpCapturePointVelocity.setIncludingFrame(desiredCapturePointVelocityToPack);
         doTimeFreezeIfNeeded(currentCapturePointPosition, time);
      }

      previousTime.set(time);
   }

   @Override
   public void initializeDoubleSupport(double initialTime, RobotSide transferToSide)
   {
      timeDelay.set(0.0);
      previousTime.set(initialTime);
      super.initializeDoubleSupport(initialTime, transferToSide);
   }

   @Override
   public void initializeSingleSupport(double initialTime, RobotSide supportSide)
   {
      timeDelay.set(0.0);
      previousTime.set(initialTime);
      super.initializeSingleSupport(initialTime, supportSide);
   }

   private void doTimeFreezeIfNeeded(FramePoint currentCapturePointPosition, double time)
   {
      computeCapturePointDistantToFreezeLine(currentCapturePointPosition, tmpCapturePointPosition, tmpCapturePointVelocity);

      if (isDone(time))
      {
         completelyFreezeTime(time);
         isTimeBeingFrozen.set(true);
      }
      else if (computeAndReturnTimeRemaining(getTimeWithDelay(time)) < 0.1 && isInDoubleSupport()
            && distanceToFreezeLine.getDoubleValue() > maxCapturePointErrorAllowedToBeginSwingPhase.getDoubleValue())
      {
         completelyFreezeTime(time);
         isTimeBeingFrozen.set(true);
      }
      else if ((distanceToFreezeLine.getDoubleValue() > maxAllowedCapturePointErrorWithoutPartialTimeFreeze.getDoubleValue()))
      {
         freezeTimeUsingFreezeTimeFactor(time);
         isTimeBeingFrozen.set(true);
      }
      else
      {
         isTimeBeingFrozen.set(false);
      }
   }

   private void freezeTimeUsingFreezeTimeFactor(double time)
   {
      timeDelay.add(freezeTimeFactor.getDoubleValue() * (time - previousTime.getDoubleValue()));
   }

   private void completelyFreezeTime(double time)
   {
      timeDelay.add(time - previousTime.getDoubleValue());
   }

   private void computeCapturePointDistantToFreezeLine(FramePoint currentCapturePointPosition, FramePoint desiredCapturePointPosition,
         FrameVector desiredCapturePointVelocity)
   {
      distanceToFreezeLine.set(CapturePointTools.computeDistanceToCapturePointFreezeLineIn2d(currentCapturePointPosition, desiredCapturePointPosition,
            desiredCapturePointVelocity));
   }

   @Override
   public void updatePlanForSingleSupportPush(FramePoint actualCapturePointPosition, double time)
   {
      timeDelay.set(0.0);
      previousTime.set(time);
      super.updatePlanForSingleSupportPush(actualCapturePointPosition, time);
   }

   @Override
   public void updatePlanForDoubleSupportPush(FramePoint actualCapturePointPosition, double time)
   {
      timeDelay.set(0.0);
      previousTime.set(time);
      super.updatePlanForDoubleSupportPush(actualCapturePointPosition, time);
   }

   public boolean getIsTimeBeingFrozen()
   {
      return isTimeBeingFrozen.getBooleanValue();
   }

   private double getTimeWithDelay(double time)
   {
      return time - timeDelay.getDoubleValue();
   }

   @Override
   public void reset(double time)
   {
      timeDelay.set(0.0);
      previousTime.set(time);
      super.reset(time);
   }
}

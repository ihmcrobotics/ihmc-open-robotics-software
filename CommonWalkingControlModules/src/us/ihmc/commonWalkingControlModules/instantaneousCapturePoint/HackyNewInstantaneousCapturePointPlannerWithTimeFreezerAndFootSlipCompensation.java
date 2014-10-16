package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class HackyNewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensation extends NewInstantaneousCapturePointPlannerWithSmoother
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final BooleanYoVariable doTimeFreezing;
   private final BooleanYoVariable doFootSlipCompensation;
   private final BooleanYoVariable isTimeBeingFrozen;
   private final DoubleYoVariable timeDelay;
   private final DoubleYoVariable capturePointPositionError;
   private final DoubleYoVariable distanceToFreezeLine;
   private final DoubleYoVariable previousTime;
   private final DoubleYoVariable freezeTimeFactor;
   private final DoubleYoVariable maxCapturePointErrorAllowedToBeginSwingPhase;
   private final YoFrameVector vectorFromActualToDesiredCapturePoint;
   private final YoFrameVector normalizedCapturePointVelocityVector;

   public HackyNewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensation(CapturePointPlannerParameters capturePointPlannerParameters,
         YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(capturePointPlannerParameters, registry, yoGraphicsListRegistry);

      timeDelay = new DoubleYoVariable("icpPlannerTimeDelayFromeFreezer", registry);
      capturePointPositionError = new DoubleYoVariable("icpPlannerCapturePointPositionError", registry);
      distanceToFreezeLine = new DoubleYoVariable("icpPlannerDistanceToFreezeLine", registry);
      freezeTimeFactor = new DoubleYoVariable("icpPlannerFreezeTimeFactor", registry);
      maxCapturePointErrorAllowedToBeginSwingPhase = new DoubleYoVariable("icpPlannerMaxCapturePointErrorAllowedToBeginSwingPhase", registry);
      previousTime = new DoubleYoVariable("icpPlannerPreviousTime", registry);
      doTimeFreezing = new BooleanYoVariable("icpPlannerDoTimeFreezing", registry);
      doFootSlipCompensation = new BooleanYoVariable("icpPlannerDoFootSlipCompensation", registry);
      isTimeBeingFrozen = new BooleanYoVariable("icpPlannerIsTimeBeingFrozen", registry);
      vectorFromActualToDesiredCapturePoint = new YoFrameVector("icpPlannerVectorFromActualToDesiredCapturePoint", worldFrame, registry);
      normalizedCapturePointVelocityVector = new YoFrameVector("icpPlannerNormalizedCapturePointVelocityVector", worldFrame, registry);

      isTimeBeingFrozen.set(false);
      timeDelay.set(0.0);
      capturePointPositionError.set(0.0);
      normalizedCapturePointVelocityVector.setToZero();
      vectorFromActualToDesiredCapturePoint.setToZero();
      distanceToFreezeLine.set(0.0);
      doTimeFreezing.set(capturePointPlannerParameters.getDoTimeFreezing());
      doFootSlipCompensation.set(capturePointPlannerParameters.getDoTimeFreezing());
      
      this.maxCapturePointErrorAllowedToBeginSwingPhase.set(capturePointPlannerParameters.getMaxInstantaneousCapturePointErrorForStartingSwing());
      this.freezeTimeFactor.set(capturePointPlannerParameters.getFreezeTimeFactor());
   }

   public void packDesiredCapturePointPositionAndVelocity(FramePoint desiredCapturePointPositionToPack, FrameVector desiredCapturePointVelocityToPack,
         double time, FramePoint currentCapturePointPosition)
   {
      super.packDesiredCapturePointPositionAndVelocity(desiredCapturePointPositionToPack, desiredCapturePointVelocityToPack, getTimeWithDelay(time));

      if (doFootSlipCompensation.getBooleanValue())
      {
         doFootSlipCompensation();
      }

      if (doTimeFreezing.getBooleanValue())
      {
         doTimeFreezeIfNeeded(currentCapturePointPosition, desiredCapturePointPositionToPack, desiredCapturePointVelocityToPack, time);
      }

   }
   
   @Override
   public void initializeDoubleSupport(FramePoint currentDesiredCapturePointPosition, FrameVector currentDesiredCapturePointVelocity,
         double initialTime, ArrayList<FramePoint> footstepList)
      {
         timeDelay.set(0.0);
         super.initializeDoubleSupport(currentDesiredCapturePointPosition, currentDesiredCapturePointVelocity, initialTime, footstepList);
      }
   
   @Override
   public void initializeSingleSupport(double initialTime, ArrayList<FramePoint> footstepList)
   {
      timeDelay.set(0.0);
      super.initializeSingleSupport(initialTime, footstepList);
   }

   private void doTimeFreezeIfNeeded(FramePoint currentCapturePointPosition, FramePoint desiredCapturePointPosition,
         FrameVector desiredCapturePointVelocity, double time)
   {
      computeCapturePointPositionErrorMagnitude(currentCapturePointPosition, desiredCapturePointPosition);
      computeCapturePointDistantToFreezeLine(currentCapturePointPosition, desiredCapturePointPosition, desiredCapturePointVelocity);
      
      if (isDone(getTimeWithDelay(time)))
      {
         completelyFreezeTime(time);
         isTimeBeingFrozen.set(true);
      }
      else if (computeAndReturnTimeRemaining(getTimeWithDelay(time)) < 0.1 && isDoubleSupport.getBooleanValue()
            && distanceToFreezeLine.getDoubleValue() > maxCapturePointErrorAllowedToBeginSwingPhase.getDoubleValue())
      {
         completelyFreezeTime(time);
         isTimeBeingFrozen.set(true);
      }
      else if ((distanceToFreezeLine.getDoubleValue() > maxCapturePointErrorAllowedToBeginSwingPhase.getDoubleValue()))
      {
         freezeTimeUsingFreezeTimeFactor(time);
         isTimeBeingFrozen.set(true);
      }
      else
      {
         isTimeBeingFrozen.set(false);
      }
      
      previousTime.set(time);
   }

   private void doFootSlipCompensation()
   {

   }

   private void freezeTimeUsingFreezeTimeFactor(double time)
   {
      if (super.computeAndReturnTimeInCurrentState(getTimeWithDelay(time)) < 0.0)
      {
         return;
      }

      timeDelay.add(freezeTimeFactor.getDoubleValue() * (time - previousTime.getDoubleValue()));
   }

   private void completelyFreezeTime(double time)
   {
      if (super.computeAndReturnTimeInCurrentState(getTimeWithDelay(time)) < 0.0)
      {
         return;
      }

      timeDelay.add(time - previousTime.getDoubleValue());
   }

   private void computeCapturePointDistantToFreezeLine(FramePoint currentCapturePointPosition, FramePoint desiredCapturePointPosition,
         FrameVector desiredCapturePointVelocity)
   {
      distanceToFreezeLine.set(CapturePointTools.computeDistanceToCapturePointFreezeLine(currentCapturePointPosition, desiredCapturePointPosition,
            desiredCapturePointVelocity));
   }

   private void computeCapturePointPositionErrorMagnitude(FramePoint currentCapturePointPosition, FramePoint desiredCapturePointPosition)
   {
      capturePointPositionError.set(currentCapturePointPosition.distance(desiredCapturePointPosition));
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
   public boolean isDone(double time)
   {
      return super.isDone(getTimeWithDelay(time));
   }
}

package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
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
      distanceToFreezeLine = new DoubleYoVariable("icpPlannerDistanceToFreezeLinre", registry);
      freezeTimeFactor = new DoubleYoVariable("icpPlannerFreezeTimeFactor", registry);
      maxCapturePointErrorAllowedToBeginSwingPhase = new DoubleYoVariable("icpPlannerMaxCapturePointErrorAllowedToBeginSwingPhase", registry);
      previousTime = new DoubleYoVariable("icpPlannerPreviousTime", registry);
      doTimeFreezing = new BooleanYoVariable("icpPlannerDoTimeFreezing", registry);
      doFootSlipCompensation = new BooleanYoVariable("icpPlannerDoFootSlipCompensation", registry);
      vectorFromActualToDesiredCapturePoint = new YoFrameVector("icpPlannerVectorFromActualToDesiredCapturePoint", worldFrame, registry);
      normalizedCapturePointVelocityVector = new YoFrameVector("icpPlannerNormalizedCapturePointVelocityVector", worldFrame, registry);

      timeDelay.set(0.0);
      capturePointPositionError.set(0.0);
      normalizedCapturePointVelocityVector.setToZero();
      vectorFromActualToDesiredCapturePoint.setToZero();
      distanceToFreezeLine.set(0.0);
      doTimeFreezing.set(capturePointPlannerParameters.getDoTimeFreezing());
      doFootSlipCompensation.set(capturePointPlannerParameters.getDoTimeFreezing());

      freezeTimeFactor.set(capturePointPlannerParameters.getFreezeTimeFactor());
   }

   public void packDesiredCapturePointPositionAndVelocity(YoFramePoint desiredCapturePointPositionToPack, YoFrameVector desiredCapturePointVelocityToPack,
         double time, YoFramePoint currentCapturePointPosition)
   {
      super.packDesiredCapturePointPositionAndVelocity(desiredCapturePointPositionToPack, desiredCapturePointVelocityToPack, time);

      if (doFootSlipCompensation.getBooleanValue())
      {
         doFootSlipCompensation();
      }

      if (doTimeFreezing.getBooleanValue())
      {
         doTimeFreezeIfNeeded(currentCapturePointPosition, desiredCapturePointPositionToPack, desiredCapturePointVelocityToPack, time);
      }

   }

   private void doTimeFreezeIfNeeded(YoFramePoint currentCapturePointPosition, YoFramePoint desiredCapturePointPosition,
         YoFrameVector desiredCapturePointVelocity, double time)
   {
      computeCapturePointPositionErrorMagnitude(currentCapturePointPosition, desiredCapturePointPosition);
      computeCapturePointDistantToFreezeLine(currentCapturePointPosition, desiredCapturePointPosition, desiredCapturePointVelocity);

      if (isDone(getTimeWithDelay(time)))
      {
         completelyFreezeTime(time);
      }
      else if (computeAndReturnTimeRemaining(getTimeWithDelay(time)) < 0.1 && isDoubleSupport.getBooleanValue()
            && distanceToFreezeLine.getDoubleValue() > maxCapturePointErrorAllowedToBeginSwingPhase.getDoubleValue()
            && Double.isNaN(distanceToFreezeLine.getDoubleValue()))
      {
         completelyFreezeTime(time);
      }
      else if ((distanceToFreezeLine.getDoubleValue() > maxCapturePointErrorAllowedToBeginSwingPhase.getDoubleValue()))
      {
         freezeTimeUsingFreezeTimeFactor(time);
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

   private void computeCapturePointDistantToFreezeLine(YoFramePoint currentCapturePointPosition, YoFramePoint desiredCapturePointPosition,
         YoFrameVector desiredCapturePointVelocity)
   {
      distanceToFreezeLine.set(CapturePointTools.computeDistanceToCapturePointFreezeLine(currentCapturePointPosition, desiredCapturePointPosition,
            desiredCapturePointVelocity));
   }

   private void computeCapturePointPositionErrorMagnitude(YoFramePoint currentCapturePointPosition, YoFramePoint desiredCapturePointPosition)
   {
      capturePointPositionError.set(currentCapturePointPosition.distance(desiredCapturePointPosition));
   }

   private double getTimeWithDelay(double time)
   {
      return time - timeDelay.getDoubleValue();
   }
}

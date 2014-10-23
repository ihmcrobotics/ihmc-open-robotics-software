package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class NewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensation extends NewInstantaneousCapturePointPlannerWithSmoother
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final EnumYoVariable<RobotSide> currentTransferToSide;
   private final BooleanYoVariable doTimeFreezing;
   private final BooleanYoVariable doFootSlipCompensation;
   private final BooleanYoVariable isTimeBeingFrozen;
   private final DoubleYoVariable timeDelay;
   private final DoubleYoVariable capturePointPositionError;
   private final DoubleYoVariable distanceToFreezeLine;
   private final DoubleYoVariable previousTime;
   private final DoubleYoVariable freezeTimeFactor;
   private final DoubleYoVariable maxCapturePointErrorAllowedToBeginSwingPhase;
   private final DoubleYoVariable maxAllowedCapturePointErrorWithoutPartialTimeFreeze;
   private final DoubleYoVariable alphaDeltaFootPosition;
   private final DoubleYoVariable changeInTransferToFootPositionMagnitude;
   private final DoubleYoVariable percentToScaleBackOnVelocity;
   private final FrameVector vectorFromActualToDesiredCapturePoint;
   private final FrameVector normalizedCapturePointVelocityVector;
   private final FramePoint currentTransferToFootLocation;
   private final FramePoint initialTransferToFootLocation;
   
   private final FramePoint tmpFramePoint;
   private final FrameVector tmpFrameVector;
   
   private final AlphaFilteredYoFrameVector changeInTransferToFootPosition;

   public NewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensation(CapturePointPlannerParameters capturePointPlannerParameters,
         YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(capturePointPlannerParameters, registry, yoGraphicsListRegistry);

      this.timeDelay = new DoubleYoVariable("icpPlannerTimeDelayFromeFreezer", registry);
      this.capturePointPositionError = new DoubleYoVariable("icpPlannerCapturePointPositionError", registry);
      this.distanceToFreezeLine = new DoubleYoVariable("icpPlannerDistanceToFreezeLine", registry);
      this.freezeTimeFactor = new DoubleYoVariable("icpPlannerFreezeTimeFactor", registry);
      this.maxCapturePointErrorAllowedToBeginSwingPhase = new DoubleYoVariable("icpPlannerMaxCapturePointErrorAllowedToBeginSwingPhase", registry);
      this.maxAllowedCapturePointErrorWithoutPartialTimeFreeze = new DoubleYoVariable("icpPlannerMaxAllowedCapturePointErrorWithoutTimeFreeze", registry);
      this.previousTime = new DoubleYoVariable("icpPlannerPreviousTime", registry);
      this.percentToScaleBackOnVelocity = new DoubleYoVariable("icpPlannerPercentToScaleBackOnVelocity", registry);
      this.alphaDeltaFootPosition = new DoubleYoVariable("icpPlannerAlphaDeltaFootPosition", registry);
      this.changeInTransferToFootPositionMagnitude = new DoubleYoVariable("icpPlannerChangeInTransferToFootPositionMagnitude", registry);
      this.doTimeFreezing = new BooleanYoVariable("icpPlannerDoTimeFreezing", registry);
      this.doFootSlipCompensation = new BooleanYoVariable("icpPlannerDoFootSlipCompensation", registry);
      this.isTimeBeingFrozen = new BooleanYoVariable("icpPlannerIsTimeBeingFrozen", registry);
      this.vectorFromActualToDesiredCapturePoint = new FrameVector(worldFrame);
      this.normalizedCapturePointVelocityVector = new FrameVector(worldFrame);
      this.currentTransferToFootLocation = new FramePoint(worldFrame);
      this.initialTransferToFootLocation = new FramePoint(worldFrame);
      this.currentTransferToSide = new EnumYoVariable<RobotSide>("icpPlannerCurrentTransferToSide", registry, RobotSide.class);
      this.tmpFramePoint = new FramePoint(worldFrame);
      this.tmpFrameVector = new FrameVector(worldFrame);
      
      this.changeInTransferToFootPosition = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("icpPlannerChangeInTransferToFootPositionFiltered", "", registry,
            alphaDeltaFootPosition, worldFrame);

      this.isTimeBeingFrozen.set(false);
      this.timeDelay.set(0.0);
      this.alphaDeltaFootPosition.set(0.65);
      this.capturePointPositionError.set(0.0);
      this.normalizedCapturePointVelocityVector.setToZero();
      this.vectorFromActualToDesiredCapturePoint.setToZero();
      this.distanceToFreezeLine.set(0.0);
      this.doTimeFreezing.set(capturePointPlannerParameters.getDoTimeFreezing());
      this.doFootSlipCompensation.set(capturePointPlannerParameters.getDoTimeFreezing());

      this.maxCapturePointErrorAllowedToBeginSwingPhase.set(capturePointPlannerParameters.getMaxInstantaneousCapturePointErrorForStartingSwing());
      this.maxAllowedCapturePointErrorWithoutPartialTimeFreeze.set(capturePointPlannerParameters.getMaxAllowedErrorWithoutPartialTimeFreeze());
      this.freezeTimeFactor.set(capturePointPlannerParameters.getFreezeTimeFactor());
   }

   public void packDesiredCapturePointPositionAndVelocity(FramePoint desiredCapturePointPositionToPack, FrameVector desiredCapturePointVelocityToPack,
         double time, FramePoint currentCapturePointPosition, FramePoint transferToFoot)
   {
      super.packDesiredCapturePointPositionAndVelocity(tmpFramePoint, tmpFrameVector, getTimeWithDelay(time));

      if (doFootSlipCompensation.getBooleanValue() && isDoubleSupport.getBooleanValue() && currentTransferToSide.getEnumValue() != null)
      {
         this.currentTransferToFootLocation.setIncludingFrame(transferToFoot);
         doFootSlipCompensation(time,tmpFramePoint,tmpFrameVector);
      }

      if (doTimeFreezing.getBooleanValue())
      {
         doTimeFreezeIfNeeded(currentCapturePointPosition,tmpFramePoint,tmpFrameVector,time);
      }

      desiredCapturePointPositionToPack.setIncludingFrame(tmpFramePoint);
      desiredCapturePointVelocityToPack.setIncludingFrame(tmpFrameVector);
      
      previousTime.set(time);
   }

   public void initializeDoubleSupport(FramePoint currentDesiredCapturePointPosition, FrameVector currentDesiredCapturePointVelocity, double initialTime,
         ArrayList<FramePoint> footstepList, RobotSide transferToSide, FramePoint transferToFootLocation)
   {
      changeInTransferToFootPosition.reset();
      timeDelay.set(0.0);
      initialTransferToFootLocation.setIncludingFrame(transferToFootLocation);
      currentTransferToSide.set(transferToSide);
      super.initializeDoubleSupport(currentDesiredCapturePointPosition, currentDesiredCapturePointVelocity, initialTime, footstepList);
   }

   @Override
   public void initializeSingleSupport(double initialTime, ArrayList<FramePoint> footstepList)
   {
      changeInTransferToFootPosition.reset();
      timeDelay.set(0.0);
      super.initializeSingleSupport(initialTime, footstepList);
   }

   private void doTimeFreezeIfNeeded(FramePoint currentCapturePointPosition, FramePoint desiredCapturePointPosition, FrameVector desiredCapturePointVelocity,
         double time)
   {
      computeCapturePointPositionErrorMagnitude(currentCapturePointPosition, desiredCapturePointPosition);
      computeCapturePointDistantToFreezeLine(currentCapturePointPosition, desiredCapturePointPosition, desiredCapturePointVelocity);

      if (isDone(time))
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

   /**
    * Really hacky method here. Need to figure out a more gooder way to do this, so 
    * it can be more gooder.
    * @param time
    */
   private void doFootSlipCompensation(double time, FramePoint capturePointPositionToPack, FrameVector capturePointVelocityToPack)
   {  
      double deltaX = currentTransferToFootLocation.getX() - initialTransferToFootLocation.getX();
      double deltaY = currentTransferToFootLocation.getY() - initialTransferToFootLocation.getY();
      
      changeInTransferToFootPosition.update(deltaX,deltaY,0.0);
      changeInTransferToFootPositionMagnitude.set(changeInTransferToFootPosition.length());
      double timeInState = super.computeAndReturnTimeInCurrentState(time);
      double timeLeft = super.computeAndReturnTimeRemaining(time);

      double percentIn = timeInState / (timeInState + timeLeft);

      percentIn = MathTools.clipToMinMax(percentIn, 0.0, 1.0);

      changeInTransferToFootPosition.scale(percentIn);
      capturePointPositionToPack.setX(capturePointPositionToPack.getX() + changeInTransferToFootPosition.getX());
      capturePointPositionToPack.setY(capturePointPositionToPack.getY() + changeInTransferToFootPosition.getY());
  
      // Scale back on the velocity if the foot slipped a lot.
      // And when coming to the end of the trajectory.
      // This is very hackish. We should make the trajectories better so we don't have
      // to do this...
      percentToScaleBackOnVelocity.set(1.0 - changeInTransferToFootPositionMagnitude.getDoubleValue() / 0.04);
      percentToScaleBackOnVelocity.set(MathTools.clipToMinMax(percentToScaleBackOnVelocity.getDoubleValue(), 0.0, 1.0));

      // At 95% in, should be at zero velocity.
      double percentToScaleDownAtEnd = 1.0 - (percentIn) / 0.95;
      percentToScaleDownAtEnd = MathTools.clipToMinMax(percentToScaleDownAtEnd, 0.0, 1.0);
      percentToScaleBackOnVelocity.set(percentToScaleBackOnVelocity.getDoubleValue() * percentToScaleDownAtEnd);
      capturePointVelocityToPack.scale(percentToScaleBackOnVelocity.getDoubleValue());
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
      distanceToFreezeLine.set(CapturePointTools.computeDistanceToCapturePointFreezeLine(currentCapturePointPosition, desiredCapturePointPosition,
            desiredCapturePointVelocity));
   }

   private void computeCapturePointPositionErrorMagnitude(FramePoint currentCapturePointPosition, FramePoint desiredCapturePointPosition)
   {
      capturePointPositionError.set(currentCapturePointPosition.distance(desiredCapturePointPosition));
   }
   
   @Override
   public void updateForSingleSupportPush(ArrayList<FramePoint> footstepList, double time)
   {
	   timeDelay.set(0.0);
	   super.updateForSingleSupportPush(footstepList, time);
   }
   
   @Override
   public void cancelPlan(double time, ArrayList<FramePoint> footstepList)
   {
	   if(isDoubleSupport.getBooleanValue())
	   {
		   timeDelay.set(0.0);
	   }
		
	   super.cancelPlan(getTimeWithDelay(time), footstepList);
   }

   public boolean getIsTimeBeingFrozen()
   {
      return isTimeBeingFrozen.getBooleanValue();
   }

   private double getTimeWithDelay(double time)
   {
      return time - timeDelay.getDoubleValue();
   }
   
   public void reset()
   {
	   changeInTransferToFootPosition.set(0.0,0.0,0.0);
	   super.reset();
   }

   @Override
   public boolean isDone(double time)
   {
      return super.isDone(getTimeWithDelay(time));
   }
}

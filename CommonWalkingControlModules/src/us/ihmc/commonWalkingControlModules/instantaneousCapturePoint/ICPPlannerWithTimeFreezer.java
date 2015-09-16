package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class ICPPlannerWithTimeFreezer extends ICPPlanner
{
   private final String namePrefix = "icpPlanner";

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
   
   public ICPPlannerWithTimeFreezer(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         CapturePointPlannerParameters capturePointPlannerParameters, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(bipedSupportPolygons, contactableFeet, capturePointPlannerParameters, parentRegistry, yoGraphicsListRegistry);

      this.timeDelay = new DoubleYoVariable(namePrefix + "TimeDelayFromFreezer", registry);
      this.capturePointPositionError = new DoubleYoVariable(namePrefix + "CapturePointPositionError", registry);
      this.distanceToFreezeLine = new DoubleYoVariable(namePrefix + "DistanceToFreezeLine", registry);
      this.freezeTimeFactor = new DoubleYoVariable(namePrefix + "FreezeTimeFactor", registry);
      this.maxCapturePointErrorAllowedToBeginSwingPhase = new DoubleYoVariable(namePrefix + "MaxCapturePointErrorAllowedToBeginSwingPhase", registry);
      this.maxAllowedCapturePointErrorWithoutPartialTimeFreeze = new DoubleYoVariable(namePrefix + "MaxAllowedCapturePointErrorWithoutTimeFreeze", registry);
      this.previousTime = new DoubleYoVariable(namePrefix + "PreviousTime", registry);
      this.doTimeFreezing = new BooleanYoVariable(namePrefix + "DoTimeFreezing", registry);
      this.isTimeBeingFrozen = new BooleanYoVariable(namePrefix + "IsTimeBeingFrozen", registry);
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
      super.packDesiredCapturePointPositionAndVelocity(desiredCapturePointPositionToPack, desiredCapturePointVelocityToPack, getTimeWithDelay(time));

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
   public void updatePlanForSingleSupportDisturbances(double time, FramePoint actualCapturePointPosition)
   {
      timeDelay.set(0.0);
      previousTime.set(time);
      super.updatePlanForSingleSupportDisturbances(time, actualCapturePointPosition);
   }

   public boolean getIsTimeBeingFrozen()
   {
      return isTimeBeingFrozen.getBooleanValue();
   }

   @Override
   public boolean isDone(double time)
   {
      return super.isDone(getTimeWithDelay(time));
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

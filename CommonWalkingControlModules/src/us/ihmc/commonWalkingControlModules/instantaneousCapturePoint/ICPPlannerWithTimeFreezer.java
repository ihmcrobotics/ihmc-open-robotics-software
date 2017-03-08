package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;

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

   private final FramePoint2d tmpCapturePointPosition;
   private final FrameVector2d tmpCapturePointVelocity;

   public ICPPlannerWithTimeFreezer(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                    CapturePointPlannerParameters capturePointPlannerParameters, YoVariableRegistry parentRegistry,
                                    YoGraphicsListRegistry yoGraphicsListRegistry)
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
      this.tmpCapturePointPosition = new FramePoint2d(worldFrame);
      this.tmpCapturePointVelocity = new FrameVector2d(worldFrame);

      this.isTimeBeingFrozen.set(false);
      this.timeDelay.set(0.0);
      this.capturePointPositionError.set(0.0);
      this.distanceToFreezeLine.set(0.0);
      this.doTimeFreezing.set(capturePointPlannerParameters.getDoTimeFreezing());

      this.maxCapturePointErrorAllowedToBeginSwingPhase.set(capturePointPlannerParameters.getMaxInstantaneousCapturePointErrorForStartingSwing());
      this.maxAllowedCapturePointErrorWithoutPartialTimeFreeze.set(capturePointPlannerParameters.getMaxAllowedErrorWithoutPartialTimeFreeze());
      this.freezeTimeFactor.set(capturePointPlannerParameters.getFreezeTimeFactor());
   }

   @Override
   public void compute(double time)
   {
      throw new RuntimeException("Use the method ICPPlannerWithTimeFreezer.compute(FramePoint2d, double) instead. If the time freeze feature is not desired, use ICPPlanner instead.");
   }

   public void compute(FramePoint2d currentCapturePointPosition, double time)
   {
      super.compute(time - timeDelay.getDoubleValue());

      if (doTimeFreezing.getBooleanValue())
      {
         super.getDesiredCapturePointPosition(tmpCapturePointPosition);
         super.getDesiredCapturePointVelocity(tmpCapturePointVelocity);
         doTimeFreezeIfNeeded(currentCapturePointPosition, time);
      }

      previousTime.set(time);
   }

   @Override
   public void initializeForTransfer(double initialTime)
   {
      timeDelay.set(0.0);
      previousTime.set(initialTime);
      super.initializeForTransfer(initialTime);
   }

   @Override
   public void initializeForSingleSupport(double initialTime)
   {
      timeDelay.set(0.0);
      previousTime.set(initialTime);
      super.initializeForSingleSupport(initialTime);
   }

   private void doTimeFreezeIfNeeded(FramePoint2d currentCapturePointPosition, double time)
   {
      computeCapturePointDistantToFreezeLine(currentCapturePointPosition, tmpCapturePointPosition, tmpCapturePointVelocity);

      if (isDone())
      {
         completelyFreezeTime(time);
         isTimeBeingFrozen.set(true);
      }
      else if (getTimeInCurrentStateRemaining() < 0.1 && isInDoubleSupport()
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

   private void computeCapturePointDistantToFreezeLine(FramePoint2d currentCapturePointPosition, FramePoint2d desiredCapturePointPosition,
                                                       FrameVector2d desiredCapturePointVelocity)
   {
      distanceToFreezeLine.set(CapturePointTools.computeDistanceToCapturePointFreezeLineIn2d(currentCapturePointPosition, desiredCapturePointPosition,
                                                                                             desiredCapturePointVelocity));
   }

   @Override
   public void updatePlanForSingleSupportDisturbances(FramePoint2d actualCapturePointPosition)
   {
      timeDelay.set(0.0);
      previousTime.set(getTimeInCurrentState());
      super.updatePlanForSingleSupportDisturbances(actualCapturePointPosition);
   }

   public boolean getIsTimeBeingFrozen()
   {
      return isTimeBeingFrozen.getBooleanValue();
   }

   @Override
   public void initializeForStanding(double time)
   {
      timeDelay.set(0.0);
      previousTime.set(time);
      super.initializeForStanding(time);
   }
}

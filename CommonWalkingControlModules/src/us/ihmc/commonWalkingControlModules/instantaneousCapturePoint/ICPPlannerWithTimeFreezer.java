package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ICPPlannerWithTimeFreezer extends ICPPlanner
{
   private final String namePrefix = "icpPlanner";

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoBoolean doTimeFreezing;
   private final YoBoolean isTimeBeingFrozen;
   private final YoDouble timeDelay;
   private final YoDouble capturePointPositionError;
   private final YoDouble distanceToFreezeLine;
   private final YoDouble previousTime;
   private final YoDouble freezeTimeFactor;
   private final YoDouble maxCapturePointErrorAllowedToBeginSwingPhase;
   private final YoDouble maxAllowedCapturePointErrorWithoutPartialTimeFreeze;

   private final FramePoint2d tmpCapturePointPosition;
   private final FrameVector2d tmpCapturePointVelocity;

   public ICPPlannerWithTimeFreezer(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                    CapturePointPlannerParameters capturePointPlannerParameters, YoVariableRegistry parentRegistry,
                                    YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(bipedSupportPolygons, contactableFeet, capturePointPlannerParameters, parentRegistry, yoGraphicsListRegistry);

      this.timeDelay = new YoDouble(namePrefix + "TimeDelayFromFreezer", registry);
      this.capturePointPositionError = new YoDouble(namePrefix + "CapturePointPositionError", registry);
      this.distanceToFreezeLine = new YoDouble(namePrefix + "DistanceToFreezeLine", registry);
      this.freezeTimeFactor = new YoDouble(namePrefix + "FreezeTimeFactor", registry);
      this.maxCapturePointErrorAllowedToBeginSwingPhase = new YoDouble(namePrefix + "MaxCapturePointErrorAllowedToBeginSwingPhase", registry);
      this.maxAllowedCapturePointErrorWithoutPartialTimeFreeze = new YoDouble(namePrefix + "MaxAllowedCapturePointErrorWithoutTimeFreeze", registry);
      this.previousTime = new YoDouble(namePrefix + "PreviousTime", registry);
      this.doTimeFreezing = new YoBoolean(namePrefix + "DoTimeFreezing", registry);
      this.isTimeBeingFrozen = new YoBoolean(namePrefix + "IsTimeBeingFrozen", registry);
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
   public void updateCurrentPlan()
   {
      timeDelay.set(0.0);
      previousTime.set(getTimeInCurrentState());
      super.updateCurrentPlan();
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

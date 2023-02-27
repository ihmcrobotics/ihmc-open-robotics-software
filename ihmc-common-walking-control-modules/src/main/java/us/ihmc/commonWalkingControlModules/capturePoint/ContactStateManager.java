package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ContactStateManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean inSingleSupport = new YoBoolean("InSingleSupport", registry);
   private final YoBoolean inStanding = new YoBoolean("InStanding", registry);
   private final YoDouble currentStateDuration = new YoDouble("CurrentStateDuration", registry);
   private final YoDouble totalStateDuration = new YoDouble("totalStateDuration", registry);
   private final YoDouble timeAtStartOfSupportSequence = new YoDouble("TimeAtStartOfSupportSequence", registry);
   private final YoDouble timeInSupportSequence = new YoDouble("TimeInSupportSequence", registry);
   private final YoDouble offsetTimeInState = new YoDouble("offsetTimeInState", registry);

   private final YoDouble adjustedTimeInSupportSequence = new YoDouble("AdjustedTimeInSupportSequence", registry);
   private final YoDouble timeAdjustment = new YoDouble("timeAdjustment", registry);
   private final YoDouble totalTimeAdjustment = new YoDouble("totalTimeAdjustment", registry);

   private final YoBoolean contactStateIsDone = new YoBoolean("ContactStateIsDone", registry);

   private final YoDouble remainingTimeInContactSequence = new YoDouble("remainingTimeInContactSequence", registry);
   private final YoDouble adjustedRemainingTimeUnderDisturbance = new YoDouble("adjustedRemainingTimeUnderDisturbance", registry);

   private final YoDouble minimumSwingDuration = new YoDouble("minSwingDurationUnderDisturbance", registry);
   private final YoDouble minimumTransferDuration = new YoDouble("minTransferDurationUnderDisturbance", registry);

   private final BooleanParameter speedUpTransferDynamicsFromError = new BooleanParameter("speedUpTransferDynamicsFromError", registry, false);
   private final BooleanParameter slowDownTransferDynamicsFromError = new BooleanParameter("slowDownTransferDynamicsFromError", registry, false);
   private final BooleanParameter speedUpSwingDynamicsFromError = new BooleanParameter("speedUpSwingDynamicsFromError", registry, false);
   private final BooleanParameter slowDownSwingDynamicsFromError = new BooleanParameter("slowDownSwingDynamicsFromError", registry, false);

   private final DoubleProvider timeAdjustmentDiscountGain = new DoubleParameter("timeAdjustmentDiscountGain", registry, 0.8);

   private final YoDouble timeAdjustmentForSwing = new YoDouble("extraTimeAdjustmentForSwing", registry);

   private final DoubleProvider time;

   public ContactStateManager(DoubleProvider time, WalkingControllerParameters walkingControllerParameters, YoRegistry parentRegistry)
   {
      this(time, walkingControllerParameters.getMinimumSwingTimeForDisturbanceRecovery(), walkingControllerParameters.getMinimumTransferTime(), parentRegistry);
   }

   public ContactStateManager(DoubleProvider time, double minimumSwingTime, double minimumTransferTime, YoRegistry parentRegistry)
   {
      this.time = time;
      this.minimumSwingDuration.set(minimumSwingTime);
      this.minimumTransferDuration.set(minimumTransferTime);

      parentRegistry.addChild(registry);
   }

   public boolean isInSingleSupport()
   {
      return inSingleSupport.getBooleanValue();
   }

   public boolean isContactStateDone()
   {
      return contactStateIsDone.getBooleanValue();
   }

   public double getTotalStateDuration()
   {
      return totalStateDuration.getDoubleValue();
   }

   public double getCurrentStateDuration()
   {
      return currentStateDuration.getDoubleValue();
   }

   public double getAdjustedTimeRemainingInCurrentSupportSequence()
   {
      return adjustedRemainingTimeUnderDisturbance.getDoubleValue();
   }

   public double getTimeRemainingInCurrentSupportSequence()
   {
      return remainingTimeInContactSequence.getDoubleValue();
   }

   public double getExtraTimeAdjustmentForSwing()
   {
      return timeAdjustmentForSwing.getDoubleValue();
   }

   public double getTimeInSupportSequence()
   {
      return adjustedTimeInSupportSequence.getDoubleValue();
   }

   public void initialize()
   {
      timeAtStartOfSupportSequence.set(time.getValue());
      timeInSupportSequence.set(0.0);
      adjustedTimeInSupportSequence.set(0.0);
      inSingleSupport.set(false);
      inStanding.set(true);
      currentStateDuration.set(Double.NaN);
      totalStateDuration.set(Double.NaN);
      offsetTimeInState.set(0.0);

      totalTimeAdjustment.set(0.0);
   }

   public void initializeForStanding()
   {

      inSingleSupport.set(false);
      inStanding.set(true);

      timeAtStartOfSupportSequence.set(time.getValue());
      timeInSupportSequence.set(0.0);
      currentStateDuration.set(Double.POSITIVE_INFINITY);
      totalStateDuration.set(Double.POSITIVE_INFINITY);
      offsetTimeInState.set(0.0);

      remainingTimeInContactSequence.set(Double.POSITIVE_INFINITY);
      adjustedRemainingTimeUnderDisturbance.set(Double.POSITIVE_INFINITY);

      adjustedTimeInSupportSequence.set(0.0);
      totalTimeAdjustment.set(0.0);

      contactStateIsDone.set(false);
   }

   public void initializeForTransfer(double transferDuration, double swingDuration)
   {
      timeAtStartOfSupportSequence.set(time.getValue());
      timeInSupportSequence.set(0.0);
      currentStateDuration.set(transferDuration);
      totalStateDuration.set(transferDuration + swingDuration);
      remainingTimeInContactSequence.set(transferDuration);
      adjustedRemainingTimeUnderDisturbance.set(transferDuration);
      offsetTimeInState.set(0.0);

      adjustedTimeInSupportSequence.set(0.0);
      totalTimeAdjustment.set(0.0);

      inStanding.set(false);
      inSingleSupport.set(false);

      contactStateIsDone.set(false);
   }

   public void initializeForSingleSupport(double transferDuration, double swingDuration)
   {
      inSingleSupport.set(true);
      inStanding.set(false);

      double stepDuration = transferDuration + swingDuration;
      timeAtStartOfSupportSequence.set(time.getValue() - transferDuration);
      timeInSupportSequence.set(transferDuration);
      currentStateDuration.set(stepDuration);
      totalStateDuration.set(stepDuration);
      remainingTimeInContactSequence.set(swingDuration);
      adjustedRemainingTimeUnderDisturbance.set(swingDuration);
      offsetTimeInState.set(transferDuration);


      adjustedTimeInSupportSequence.set(transferDuration);
      totalTimeAdjustment.set(0.0);

      contactStateIsDone.set(false);
   }

   public void initializeForTransferToStanding(double finalTransferDuration )
   {
      timeAtStartOfSupportSequence.set(time.getValue());
      timeInSupportSequence.set(0.0);
      currentStateDuration.set(finalTransferDuration);
      totalStateDuration.set(finalTransferDuration);
      remainingTimeInContactSequence.set(finalTransferDuration);
      adjustedRemainingTimeUnderDisturbance.set(finalTransferDuration);
      offsetTimeInState.set(0.0);

      adjustedTimeInSupportSequence.set(0.0);
      totalTimeAdjustment.set(0.0);

      inSingleSupport.set(false);
      inStanding.set(false);

      contactStateIsDone.set(false);
   }

   public void updateTimeInState(DoubleProvider timeShiftProvider, boolean shouldAdjustTimeFromTrackingError)
   {
      // If this condition is false we are experiencing a late touchdown or a delayed liftoff. Do not advance the time in support sequence!
      if (inStanding.getBooleanValue() || !contactStateIsDone.getBooleanValue())
         timeInSupportSequence.set(time.getValue() - timeAtStartOfSupportSequence.getValue());

      remainingTimeInContactSequence.set(currentStateDuration.getDoubleValue() - timeInSupportSequence.getDoubleValue());

      if (shouldAdjustTimeFromTrackingError)
      {
         double minDuration = (inSingleSupport.getBooleanValue() ? minimumSwingDuration.getDoubleValue() : minimumTransferDuration.getDoubleValue());
         double maxTotalAdjustment = Math.max(currentStateDuration.getDoubleValue() - offsetTimeInState.getDoubleValue() - minDuration, 0.0);

         double remainingTimeAfterAdjustment = Math.max(remainingTimeInContactSequence.getDoubleValue() - totalTimeAdjustment.getDoubleValue(), 0.0);
         double maxAdjustment = Math.min(remainingTimeAfterAdjustment, maxTotalAdjustment - totalTimeAdjustment.getDoubleValue());
         double minAdjustment = -remainingTimeAfterAdjustment;

         double proposedAdjustment = computeTimeAdjustmentForDynamicsBasedOnState(timeShiftProvider);
         proposedAdjustment = MathTools.clamp(proposedAdjustment, minAdjustment, maxAdjustment);
         if (proposedAdjustment > 0.0)
         {
            if ((isInSingleSupport() && speedUpSwingDynamicsFromError.getValue()) || (!isInSingleSupport() && speedUpTransferDynamicsFromError.getValue()))
            {
               timeAdjustment.set(timeAdjustmentDiscountGain.getValue() * proposedAdjustment);
               timeAdjustmentForSwing.set(0.0);
            }
            else
            {
               timeAdjustmentForSwing.set(proposedAdjustment);
               timeAdjustment.set(0.0);
            }
         }
         else
         {
            if ((isInSingleSupport() && slowDownSwingDynamicsFromError.getValue()) || (!isInSingleSupport() && slowDownTransferDynamicsFromError.getValue()))
            {
               timeAdjustment.set(timeAdjustmentDiscountGain.getValue() * proposedAdjustment);
               timeAdjustmentForSwing.set(0.0);
            }
            else
            {
               timeAdjustmentForSwing.set(proposedAdjustment);
               timeAdjustment.set(0.0);
            }
         }
      }
      else
      {
         timeAdjustmentForSwing.set(0.0);
         timeAdjustment.set(0.0);
      }

      totalTimeAdjustment.add(timeAdjustment.getValue());


      adjustedTimeInSupportSequence.set(MathTools.clamp(timeInSupportSequence.getValue() + totalTimeAdjustment.getValue(), 0.0, currentStateDuration.getDoubleValue()));

      adjustedRemainingTimeUnderDisturbance.set(currentStateDuration.getDoubleValue() - adjustedTimeInSupportSequence.getDoubleValue());

      contactStateIsDone.set(adjustedTimeInSupportSequence.getValue() >= currentStateDuration.getValue());
   }

   private double computeTimeAdjustmentForDynamicsBasedOnState(DoubleProvider timeShiftProvider)
   {
      if (inStanding.getValue())
      {
         return 0.0;
      }
      else
      {
         double timeAdjustmentToApply = timeShiftProvider.getValue();
         double timeIntoState = adjustedTimeInSupportSequence.getDoubleValue();
         double timeRemainingInState = currentStateDuration.getDoubleValue() - timeIntoState;
         timeAdjustmentToApply = MathTools.clamp(timeAdjustmentToApply, -timeIntoState, timeRemainingInState);
         if (Math.abs(timeAdjustmentToApply) < 1e-3)
            timeAdjustmentToApply = 0.0;
         return Double.isNaN(timeAdjustmentToApply) ? 0.0 : timeAdjustmentToApply;
      }
   }
}

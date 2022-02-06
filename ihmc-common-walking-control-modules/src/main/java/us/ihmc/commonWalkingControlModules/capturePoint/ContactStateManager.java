package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commons.MathTools;
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
   private final YoDouble timeInSupportSequence = new YoDouble("TimeInSupportSequence", registry);

   private final YoDouble adjustedTimeInSupportSequence = new YoDouble("AdjustedTimeInSupportSequence", registry);
   private final YoDouble timeAdjustment = new YoDouble("timeAdjustment", registry);
   private final YoDouble totalTimeAdjustment = new YoDouble("totalTimeAdjustment", registry);

   private final YoBoolean contactStateIsDone = new YoBoolean("ContactStateIsDone", registry);



   private final double controlDt;

   public ContactStateManager(double controlDt, YoRegistry parentRegistry)
   {
      this.controlDt = controlDt;

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

   public double getAdjustedTimeRemainingInCurrentSupportSequence()
   {
      return currentStateDuration.getDoubleValue() - adjustedTimeInSupportSequence.getDoubleValue();
   }

   public double getTimeRemainingInCurrentSupportSequence()
   {
      return currentStateDuration.getDoubleValue() - timeInSupportSequence.getDoubleValue();
   }

   public double getTimeInSupportSequence()
   {
      return adjustedTimeInSupportSequence.getDoubleValue();
   }

   public void initialize()
   {
      timeInSupportSequence.set(0.0);
      adjustedTimeInSupportSequence.set(0.0);
      inSingleSupport.set(false);
      inStanding.set(true);
      currentStateDuration.set(Double.NaN);
      totalStateDuration.set(Double.NaN);

      totalTimeAdjustment.set(0.0);
   }

   public void initializeForStanding()
   {
      inSingleSupport.set(false);
      inStanding.set(true);

      timeInSupportSequence.set(0.0);
      currentStateDuration.set(Double.POSITIVE_INFINITY);
      totalStateDuration.set(Double.POSITIVE_INFINITY);

      adjustedTimeInSupportSequence.set(0.0);
      totalTimeAdjustment.set(0.0);

      contactStateIsDone.set(false);
   }

   public void initializeForTransfer(double transferDuration, double swingDuration)
   {
      timeInSupportSequence.set(0.0);
      currentStateDuration.set(transferDuration);
      totalStateDuration.set(transferDuration + swingDuration);

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
      timeInSupportSequence.set(transferDuration);
      currentStateDuration.set(stepDuration);
      totalStateDuration.set(stepDuration);

      adjustedTimeInSupportSequence.set(transferDuration);
      totalTimeAdjustment.set(0.0);

      contactStateIsDone.set(false);
   }

   public void initializeForTransferToStanding(double finalTransferDuration )
   {
      timeInSupportSequence.set(0.0);
      currentStateDuration.set(finalTransferDuration);
      totalStateDuration.set(finalTransferDuration);

      adjustedTimeInSupportSequence.set(0.0);
      totalTimeAdjustment.set(0.0);

      inSingleSupport.set(false);
      inStanding.set(false);

      contactStateIsDone.set(false);
   }

   public void updateTimeInState(DoubleProvider timeShiftProvider)
   {
      // If this condition is false we are experiencing a late touchdown or a delayed liftoff. Do not advance the time in support sequence!
      if (inStanding.getBooleanValue() || !contactStateIsDone.getBooleanValue())
         timeInSupportSequence.add(controlDt);

      timeAdjustment.set(computeTimeAdjustmentForDynamicsBasedOnState(timeShiftProvider));
      totalTimeAdjustment.add(timeAdjustment.getValue());

      adjustedTimeInSupportSequence.set(MathTools.clamp(timeInSupportSequence.getValue() + totalTimeAdjustment.getValue(), 0.0, currentStateDuration.getDoubleValue()));

      contactStateIsDone.set(adjustedTimeInSupportSequence.getValue() >= currentStateDuration.getValue());
   }

   public double estimateTimeRemainingForSwingUnderDisturbance(DoubleProvider timeShiftProvider)
   {
      double timeRemainingInCurrentState = getAdjustedTimeRemainingInCurrentSupportSequence();
//      if (isContactStateDone())
//         return 0.0;

      // FIXME is this necessary now?
      double deltaTimeToBeAccounted = timeShiftProvider.getValue();

      if (Double.isNaN(deltaTimeToBeAccounted))
         return 0.0;

      double estimatedTimeRemaining = timeRemainingInCurrentState - deltaTimeToBeAccounted;
      estimatedTimeRemaining = MathTools.clamp(estimatedTimeRemaining, 0.0, Double.POSITIVE_INFINITY);

      return estimatedTimeRemaining;
   }

   private double computeTimeAdjustmentForDynamicsBasedOnState(DoubleProvider timeShiftProvider)
   {
      if (inStanding.getValue())
      {
         return 0.0;
      }
      else
      {
         double timeAdjustmentToApply = 0.5 * timeShiftProvider.getValue();
         double timeIntoState = adjustedTimeInSupportSequence.getDoubleValue();
         double timeRemainingInState = currentStateDuration.getDoubleValue() - timeIntoState;
         timeAdjustmentToApply = MathTools.clamp(timeAdjustmentToApply, -timeIntoState, timeRemainingInState);
         if (Math.abs(timeAdjustmentToApply) < 1e-3)
            timeAdjustmentToApply = 0.0;
         return Double.isNaN(timeAdjustmentToApply) ? 0.0 : timeAdjustmentToApply;
      }
   }
}

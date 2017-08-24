package us.ihmc.commonWalkingControlModules.configurations;

public class DynamicReachabilityParameters
{
   /**
    * This is the change in height during the step which signifies stepping up.
    * When stepping up, the leading leg is allowed to bend more.
    *
    * @return step up (m).
    */
   public double getThresholdForStepUp()
   {
      return 0.10;
   }

   /**
    * This is the change in height during the step which signifies stepping down.
    * When stepping down, the trailing leg is allowed to bend more.
    *
    * @return step down (m).
    */
   public double getThresholdForStepDown()
   {
      return -0.10;
   }

   /**
    * This is the maximum desired knee bend that we try to adjust the dynamic plan to achieve.
    *
    * @return knee bend (rad).
    */
   public double getMaximumDesiredKneeBend()
   {
      return 0.25;
   }

   /**
    * This is the maximum number of adjustments that the solver attempts to achieve the desired adjustment.
    *
    * @return number of adjustments
    */
   public int getMaximumNumberOfCoMAdjustments()
   {
      return 3;
   }

   /**
    * This sets whether or not the solver attempts to use "higher order" steps. These tends to have
    * very small gradients, so they are typically not used.
    *
    * @return whether or not to use higher order steps.
    */
   public boolean useHigherOrderSteps()
   {
      return false;
   }

   /**
    * This is the percent of the transfer duration that we modify the ICP plan by to calculate the gradient.
    *
    * @return percent (0.0 to 1.0)
    */
   public double getPercentOfTransferDurationToCalculateGradient()
   {
      return 0.4;
   }

   /**
    * This is the percent of the swing duration that we modify the ICP plan by to calculate the gradient.
    *
    * @return percent (0.0 to 1.0)
    */
   public double getPercentOfSwingDurationToCalculateGradient()
   {
      return 0.4;
   }

   /**
    * This is the additional adjustment added on to the required adjustment, as a percentage of the
    * width of the reachable region.
    *
    * @return percent (0.0 to 1.0)
    */
   public double getRequiredAdjustmentSafetyFactor()
   {
      return 0.0;
   }

   /**
    * This gain modified the required adjustment used by the solver based on the achieved adjustment and the
    * actual desired adjustment.
    *
    * @return feedback gain.
    */
   public double getRequiredAdjustmentFeedbackGain()
   {
      return 0.1;
   }

   /**
    * Minimum allowable transfer duration computed by the solver.
    */
   public double getMinimumTransferDuration()
   {
      return 0.20;
   }

   /**
    * Maximum allowable transfer duration computed by the solver.
    */
   public double getMaximumTransferDuration()
   {
      return 5.0;
   }

   /**
    * Minimum allowable swing duration computed by the solver.
    */
   public double getMinimumSwingDuration()
   {
      return 0.4;
   }

   /**
    * Maximum allowable swing duration computed by the solver.
    */
   public double getMaximumSwingDuration()
   {
      return 10.0;
   }

   /**
    * Minimum allowable initial transfer duration computed by the solver.
    */
   public double getMinimumInitialTransferDuration()
   {
      return 0.4 * getMinimumTransferDuration();
   }

   /**
    * Minimum allowable ending transfer duration computed by the solver.
    */
   public double getMinimumEndTransferDuration()
   {
      return 0.08;
   }

   /**
    * Minimum allowable initial swing duration computed by the solver.
    */
   public double getMinimumInitialSwingDuration()
   {
      return 0.15;
   }

   /**
    * Minimum allowable ending swing duration computed by the solver.
    */
   public double getMinimumEndSwingDuration()
   {
      return 0.15;
   }

   /**
    * This is the weight placed on achieving the desired adjustment in the QP Solver.
    */
   public double getConstraintWeight()
   {
      return 10000.0;
   }

   /**
    * This weight penalizes adjustments perpendicular to the required adjustment to achieved dynamic feasibility.
    * That is, it penalizes adjustment of the CoM position perpendicular to the stepping direction.
    */
   public double getPerpendicularWeight()
   {
      return 0.1;
   }

   /**
    * This is the weight penalizing adjusting the swing entry and exit duration in the solver.
    */
   public double getSwingAdjustmentWeight()
   {
      return 20.0;
   }

   /**
    * This is the weight penalizing adjusting the transfer entry and exit duration in the solver.
    */
   public double getTransferAdjustmentWeight()
   {
      return 1.0;
   }

   /**
    * This is the weight penalizing adjusting the swing duration in the solver.
    */
   public double getTotalSwingAdjustmentWeight()
   {
      return 0.1;
   }

   /**
    * This is the weight penalizing adjusting the transfer duration in the solver.
    */
   public double getTotalTransferAdjustmentWeight()
   {
      return 0.01;
   }

   public double getTransferEqualAdjustmentWeight()
   {
      return 50.0;
   }

   public double getSwingEqualAdjustmentWeight()
   {
      return 5.0;
   }
}

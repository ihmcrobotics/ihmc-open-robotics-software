package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

/**
 * Parameters to tune the ICP Optimization based controller for each robot.
 * The ICP Optimization based controller encodes the ICP plan based on the upcoming footsteps, and can either do control
 * with adjusting the feet or without adjusting the feet, using a feedback-based convex optimization.
 */
public abstract class ICPOptimizationParameters
{
   /**
    * The maximum number of footsteps that can be considered by the controller. The variable {@link #numberOfFootstepsToConsider()} is clipped to this value.
    * It is also used to instantiate all the yo variable lists.
    */
   public int getMaximumNumberOfFootstepsToConsider()
   {
      return 5;
   }

   /**
    * How many footsteps the optimization considers for adjustment.
    * 1 footstep seems to be good.
    * With a penalization on the dynamics themselves, future steps show little effect on the current dynamics.
    */
   public abstract int numberOfFootstepsToConsider();

   /**
    * The weight for tracking the desired footsteps.
    * Setting this weight fairly high ensures that the footsteps will only be adjusted when the CMP control authority has been saturated.
    */
   public abstract double getForwardFootstepWeight();

   /**
    * The weight for tracking the desired footsteps.
    * Setting this weight fairly high ensures that the footsteps will only be adjusted when the CMP control authority has been saturated.
    */
   public abstract double getLateralFootstepWeight();

   /**
    * Penalization on changes in the footstep location solution between control ticks.
    * This weight is normalized by the control DT.
    */
   public abstract double getFootstepRegularizationWeight();

   /**
    * The weight for tracking the nominal desired CMP.
    * This weight penalizes using a large amount of CMP control.
    * Setting this weight high will make the robot behave similar to using point feet control / minimal ankle torques and angular momentum.
    */
   public abstract double getFeedbackForwardWeight();

   /**
    * The weight for tracking the nominal desired CMP.
    * This weight penalizes using a large amount of CMP control.
    * Setting this weight high will make the robot behave similar to using point feet control / minimal ankle torques and angular momentum.
    */
   public abstract double getFeedbackLateralWeight();

   /**
    * Penalization on changes feedback CMP between control ticks.
    * This weight is normalized by the control DT.
    */
   public abstract double getFeedbackRegularizationWeight();

   /**
    * Feedback gain for ICP error parallel to the desired ICP dynamics.
    */
   public abstract double getFeedbackParallelGain();

   /**
    * Feedback gain for ICP error orthogonal to the desired ICP dynamics.
    * When the desired ICP dynamics are zero, this is the gain that is used for all directions.
    */
   public abstract double getFeedbackOrthogonalGain();

   /**
    * Weight on the slack variable introduced for the ICP dynamics.
    * This slack variable is required for the CMP to be constrained inside the support polygon when not using step adjustment,
    * and the step lengths to be constrained when allowing step adjustment.
    */
   public abstract double getDynamicRelaxationWeight();

   /**
    * Modifier to reduce the dynamic relaxation penalization when in double support.
    * This is introduced to improve the problem feasibility when switching between contact states.
    */
   public abstract double getDynamicRelaxationDoubleSupportWeightModifier();

   /**
    * Enabling this boolean causes the {@link #getFootstepRegularizationWeight()} to be increased when approaching the end of the step.
    * This acts as a way to cause the solution to "lock in" near the step end.
    */
   public abstract boolean scaleStepRegularizationWeightWithTime();

   /**
    * Enabling this boolean causes the {@link #getFeedbackWeight()} to be decreased with an increasing feedback weight.
    * This allows tuning of the tendency to use feedback vs. step adjustment to be separated from the feedback controller.
    */
   public abstract boolean scaleFeedbackWeightWithGain();

   /**
    * Enabling this boolean causes {@link #getFootstepWeight()} to be decreased sequentially for upcoming steps.
    * Using this increases the likelihood of adjusting future steps, as well.
    */
   public abstract boolean scaleUpcomingStepWeights();

   /**
    * Enabling this boolean enables the use of feedback regularization, found in {@link #getFeedbackRegularizationWeight()}.
    */
   public abstract boolean useFeedbackRegularization();

   /**
    * Enabling this boolean enables the use step adjustment for stabilization.
    */
   public abstract boolean useStepAdjustment();

   /**
    * Enabling this boolean enables the use of step adjustment regularization, found in {@link #getFootstepRegularizationWeight()}.
    */
   public abstract boolean useFootstepRegularization();

   /**
    * The minimum value to allow the footstep weight {@link #getFootstepWeight()} to be set to.
    * Ensures that the costs remain positive-definite, and improves the solution numerics.
    */
   public abstract double getMinimumFootstepWeight();

   /**
    * The minimum value to allow the feedback weight {@link #getFeedbackWeight()} to be set to.
    * Ensures that the costs remain positive-definite, and improves the solution numerics.
    */
   public abstract double getMinimumFeedbackWeight();

   /**
    * The minimum value to use for the time remaining when computing the recursion multipliers.
    * This makes sure the problem maintains a "nice" form.
    */
   public abstract double getMinimumTimeRemaining();

   /**
    * Maximum forward distance the CMP is allowed to exit the support polygon.
    * Defined in the midZUpFrame when in double support, and the soleZUpFrame when in single support.
    * Exiting the support polygon is achieved by using angular momentum.
    * This should be used sparingly.
    */
   public abstract double getDoubleSupportMaxCMPForwardExit();

   /**
    * Maximum lateral distance the CMP is allowed to exit the support polygon.
    * Defined in the midZUpFrame when in double support, and the soleZUpFrame when in single support.
    * Exiting the support polygon is achieved by using angular momentum.
    * This should be used sparingly.
    */

   public abstract double getDoubleSupportMaxCMPLateralExit();
   /**
    * Maximum forward distance the CMP is allowed to exit the support polygon.
    * Defined in the midZUpFrame when in double support, and the soleZUpFrame when in single support.
    * Exiting the support polygon is achieved by using angular momentum.
    * This should be used sparingly.
    */
   public abstract double getSingleSupportMaxCMPForwardExit();

   /**
    * Maximum lateral distance the CMP is allowed to exit the support polygon.
    * Defined in the midZUpFrame when in double support, and the soleZUpFrame when in single support.
    * Exiting the support polygon is achieved by using angular momentum.
    * This should be used sparingly.
    */
   public abstract double getSingleSupportMaxCMPLateralExit();

   /**
    * Deadband on the step adjustment.
    * When the adjustment is within the deadband, it is set to zero.
    * When it is outside the deadband, the deadband is subtracted from it.
    */
   public abstract double getAdjustmentDeadband();

   /**
    * Represents the amount of adjustment to define as big
    */
   public boolean useDifferentSplitRatioForBigAdjustment()
   {
      return true;
   }

   /**
    * Represents the amount of adjustment to define as big
    */
   public double getMagnitudeForBigAdjustment()
   {
      return 0.2;
   }

   /**
    * Represents in percent of the current double support duration, how much time the transfer will spend before reaching the next entry CMP.
    * The returned value should be between 0.0 and 1.0:
    * <li> 0.0 is equivalent to spend the entire double support on the initial CMP (last entry CMP if using one CMP per support, last exit CMP otherwise), </li>
    * <li> 1.0 is equivalent to spend the entire double support on the next entry CMP. </li>
    * <p> A value close to 0.5 is preferable. </p>
    */
   public double getDoubleSupportSplitFractionForBigAdjustment()
   {
      return 0.5;
   }

   /**
    * Represents the minimum time in transfer before reaching the next entry CMP.
    */
   public double getMinimumTimeOnInitialCMPForBigAdjustment()
   {
      return 0.15;
   }

   /**
    * This method sets what the minimum change in the current footstep is allowed to be.
    * Works in tandem with the footstep regularization parameter.
    */
   public double getFootstepSolutionResolution()
   {
      return 0.015;
   }

   /**
    * @return The maximum lateral limit that the swing foot can reach w.r.t. the stance foot.
    */
   public double getLateralReachabilityOuterLimit()
   {
      return 0.5;
   }

   /**
    * @return The minimum lateral limit that the swing foot can reach w.r.t. the stance foot.
    */
   public double getLateralReachabilityInnerLimit()
   {
      return 0.1;
   }

   /**
    * @return The forward limit that the swing foot can reach w.r.t. the stance foot.
    */
   public double getForwardReachabilityLimit()
   {
      return 0.5;
   }

   /**
    * @return The backward limit that the swing foot can reach w.r.t. the stance foot.
    */
   public double getBackwardReachabilityLimit()
   {
      return -0.3;
   }
}

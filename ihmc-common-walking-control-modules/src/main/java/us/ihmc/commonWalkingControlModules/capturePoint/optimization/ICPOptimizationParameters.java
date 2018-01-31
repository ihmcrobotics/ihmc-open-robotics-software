package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

/**
 * Parameters to tune the ICP Optimization based controller for each robot.
 * The ICP Optimization based controller encodes the ICP plan based on the upcoming footsteps, and can either do control
 * with adjusting the feet or without adjusting the feet, using a feedback-based convex optimization.
 */
public abstract class ICPOptimizationParameters
{
   /**
    * Specifies a scaling factor on the amount of footstep adjustment. By increasing this past 1.0, the footstep will be adjusted more than explicitly necessary.
    */
   public double getFootstepAdjustmentSafetyFactor()
   {
      return 1.0;
   }

   /**
    * The weight for tracking the desired footsteps.
    * Setting this weight fairly high ensures that the footsteps will only be adjusted when the CoP control authority has been saturated.
    */
   public abstract double getForwardFootstepWeight();

   /**
    * The weight for tracking the desired footsteps.
    * Setting this weight fairly high ensures that the footsteps will only be adjusted when the CoP control authority has been saturated.
    */
   public abstract double getLateralFootstepWeight();

   /**
    * Penalization on changes in the footstep location solution between control ticks.
    * This weight is normalized by the control DT.
    */
   public abstract double getFootstepRateWeight();

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
   public abstract double getFeedbackRateWeight();

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
    * This slack variable is required for the CoP to be constrained inside the support polygon when not using step adjustment,
    * and the step lengths to be constrained when allowing step adjustment.
    */
   public abstract double getDynamicsObjectiveWeight();

   /**
    * Modifier to reduce the dynamic relaxation penalization when in double support.
    * This is introduced to improve the problem feasibility when switching between contact states.
    */
   public abstract double getDynamicsObjectiveDoubleSupportWeightModifier();

   /**
    * Weight on the use of angular momentum minimization.
    * This is only utilized when it is specified to use angular momentum in the feedback controller.
    */
   public abstract double getAngularMomentumMinimizationWeight();

   /**
    * Enabling this boolean causes the {@link #getFootstepRateWeight()} to be increased when approaching the end of the step.
    * This acts as a way to cause the solution to "lock in" near the step end.
    */
   public abstract boolean scaleStepRateWeightWithTime();

   /**
    * Enabling this boolean causes the {@link #getFeedbackForwardWeight()} and {@link #getFeedbackLateralWeight()} to be decreased
    * with an increasing feedback weight. This allows tuning of the tendency to use feedback vs. step adjustment to be separated from
    * the feedback controller.
    */
   public abstract boolean scaleFeedbackWeightWithGain();

   /**
    * Enabling this boolean enables the use of feedback rate, found in {@link #getFeedbackRateWeight()}.
    */
   public abstract boolean useFeedbackRate();

   /**
    * Enabling this boolean enables the use step adjustment for stabilization.
    */
   public abstract boolean allowStepAdjustment();

   /**
    * Enabling this boolean allows the CMP to exit the support polygon.
    * The CoP will still be constrained to lie inside the support polygon, however.
    */
   public abstract boolean useAngularMomentum();

   /**
    * Enabling this boolean enables the use of step adjustment rate, found in {@link #getFootstepRateWeight()}.
    */
   public abstract boolean useFootstepRate();

   /**
    * The minimum value to allow the footstep weight {@link #getForwardFootstepWeight()} and {@link #getLateralFootstepWeight()} to be set to.
    * Ensures that the costs remain positive-definite, and improves the solution numerics.
    */
   public abstract double getMinimumFootstepWeight();

   /**
    * The minimum value to allow the feedback weight {@link #getFeedbackForwardWeight()} and {@link #getFeedbackLateralWeight()} to be set to.
    * Ensures that the costs remain positive-definite, and improves the solution numerics.
    */
   public abstract double getMinimumFeedbackWeight();

   /**
    * The minimum value to use for the time remaining when computing the recursion multipliers.
    * This makes sure the problem maintains a "nice" form.
    */
   public abstract double getMinimumTimeRemaining();

   /**
    * Deadband on the step adjustment.
    * When the adjustment is within the deadband, it is set to zero.
    * When it is outside the deadband, the deadband is subtracted from it.
    */
   public abstract double getAdjustmentDeadband();

   /**
    * This method sets what the minimum change in the current footstep is allowed to be.
    * Works in tandem with the footstep rate parameter.
    */
   public double getFootstepSolutionResolution()
   {
      return 0.015;
   }

   /**
    * Sets the minimum distance inside the support polygon for the CoP to be located.
    */
   public double getSafeCoPDistanceToEdge()
   {
      return 0.002;
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

   /**
    * Sets whether or not to use a warm start in the active set solver. This exploits that the active set doesn't change often.
    * @return Whether or not to use a warm start in the solver
    */
   public boolean useWarmStartInSolver()
   {
      return false;
   }

   /**
    * Specifies whether or not to use the ICP control polygon for the CMP, rather than the actual support polygon.
    */
   public boolean getUseICPControlPolygons()
   {
      return true;
   }

   /**
    * Limits the reachability polygon constraint so that the foot cannot be retracted once it has started adjusting.
    */
   public boolean getLimitReachabilityFromAdjustment()
   {
      return false;
   }

   /**
    * Specifies whether or not to increase the cost of using angular momentum based on the amount of angular momentum used.
    */
   public boolean getUseAngularMomentumIntegrator()
   {
      return true;
   }

   /**
    * Defines the integral gain for the angular momentum integrator, as used in {@link #getUseAngularMomentumIntegrator()}.
    */
   public double getAngularMomentumIntegratorGain()
   {
      return 50.0;
   }

   /**
    * Defines the integral leak ratio for the angular momentum integrator, as used in {@link #getUseAngularMomentumIntegrator()}.
    */
   public double getAngularMomentumIntegratorLeakRatio()
   {
      return 0.92;
   }

   /**
    * Specifies the transfer split fraction to use for the ICP value recursion multiplier.
    */
   public double getTransferSplitFraction()
   {
      return 0.3;
   }

   /**
    * Sets whether or not to account for the used angular momentum when computing the amount of step adjustment.
    */
   public boolean considerAngularMomentumInAdjustment()
   {
      return true;
   }

   /**
    * Sets whether or not to account for the used CoP feedback when computing the amount of step adjustment.
    */
   public boolean considerFeedbackInAdjustment()
   {
      return true;
   }

   /**
    * Sets whether or not the ICP optimization algorithm will consider planar regions.
    */
   public boolean usePlanarRegionConstraints()
   {
      return false;
   }

   /**
    * Sets whether or not the ICP optimization algorithm will switch the planar region if it starts to lose balance.
    */
   public boolean switchPlanarRegionConstraintsAutomatically()
   {
      return true;
   }
}

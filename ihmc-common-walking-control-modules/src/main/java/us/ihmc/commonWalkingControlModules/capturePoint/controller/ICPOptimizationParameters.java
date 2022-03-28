package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;

/**
 * Parameters to tune the ICP Optimization based controller for each robot.
 * The ICP Optimization based controller encodes the ICP plan based on the upcoming footsteps, and can either do control
 * with adjusting the feet or without adjusting the feet, using a feedback-based convex optimization.
 */
public abstract class ICPOptimizationParameters
{
   /**
    * Specifies the amount of ICP error (the 2D distance in XY from desired to current) that is required for the controller to consider step adjustment.
    */
   public double getMinICPErrorForStepAdjustment()
   {
      return 0.0;
   }

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
    * Penalization on changes in the feedback CoP and CMP between control ticks.
    * This weight is normalized by the control DT.
    */
   public double getCoPCMPFeedbackRateWeight()
   {
      return 0.0;
   }

   /**
    * Penalization on changes in the total feedback between control ticks.
    * This weight is normalized by the control DT.
    */
   public abstract double getFeedbackRateWeight();

   /**
    * Gains for the proportional ICP controller that is encoded into the optimization. Also includes gains for the smart
    * integrator that is used when the controller is stuck.
    */
   public abstract ICPControlGainsReadOnly getICPFeedbackGains();

   /**
    * Sets whether the integration gains returned by {@link #getICPFeedbackGains()} is used to perform a smart integration when the robot is stuck.
    */
   public boolean useSmartICPIntegrator()
   {
      return false;
   }

   /**
    * Sets the maximum ICP velocity for it to be considered "stuck".
    */
   public double getICPVelocityThresholdForStuck()
   {
      return 0.01;
   }

   /**
    * Weight on the slack variable introduced for the ICP dynamics.
    * This slack variable is required for the CoP to be constrained inside the support polygon when not using step adjustment,
    * and the step lengths to be constrained when allowing step adjustment.
    */
   public abstract double getDynamicsObjectiveWeight();

   /**
    * Weight on the use of angular momentum minimization.
    * This is only utilized when it is specified to use angular momentum in the feedback controller.
    */
   public abstract double getAngularMomentumMinimizationWeight();

   /**
    * Enabling this boolean causes the {@link #getFeedbackForwardWeight()} and {@link #getFeedbackLateralWeight()} to be decreased
    * with an increasing feedback weight. This allows tuning of the tendency to use feedback vs. step adjustment to be separated from
    * the feedback controller.
    */
   public abstract boolean scaleFeedbackWeightWithGain();

   /**
    * Enabling this boolean enables the use step adjustment for stabilization.
    */
   public abstract boolean allowStepAdjustment();

   /**
    * Enabling this boolean allows modifying the CMP offset from the CoP in the optimization.
    */
   public boolean useCMPFeedback()
   {
      return true;
   }

   /**
    * Enabling this boolean allows the CMP to exit the support polygon.
    * The CoP will still be constrained to lie inside the support polygon, however.
    */
   public abstract boolean useAngularMomentum();


   public double getFeedbackDirectionWeight()
   {
      return 0.0;
   }


   /**
    * Deadband on the step adjustment.
    * When the adjustment is within the deadband, it is set to zero.
    * When it is outside the deadband, the deadband is subtracted from it.
    */
   public abstract double getAdjustmentDeadband();


   /**
    * Sets the minimum distance inside the support polygon for the CoP to be located.
    */
   public double getSafeCoPDistanceToEdge()
   {
      return 0.002;
   }

   /**
    * Specifies whether or not to use the ICP control polygon for the CMP, rather than the actual support polygon.
    */
   public boolean getUseICPControlPolygons()
   {
      return true;
   }

   /**
    * Specifies the transfer split fraction to use for the ICP value recursion multiplier. This value is added to the time remaining
    * to compute the recursion multiplier. Increasing this value effectively causes more step adjustment to occur.
    */
   public double getTransferSplitFraction()
   {
      return 0.1;
   }

   /**
    * Specifies the maximum duration that can be included in the footstep multiplier by the {@link #getTransferSplitFraction()}.
    * This is useful when the robot by default has long split fractions.
    */
   public double maximumTimeFromTransferInFootstepMultiplier()
   {
      return 0.1;
   }
   
   /**
    * Specifies the minimum footstep multiplier that the robot will use to compute the desired step adjustment. This is
    * particularly useful when walking slowly or when recovering early in the, to avoid extremely large
    * footstep adjustment magnitudes.
    */
   public double getMinimumFootstepMultiplier()
   {
      return 0.33;
   }


}

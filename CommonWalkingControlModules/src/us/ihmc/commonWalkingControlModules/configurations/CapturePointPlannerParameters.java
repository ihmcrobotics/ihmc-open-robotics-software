package us.ihmc.commonWalkingControlModules.configurations;

/**
 * Parameters to tune the ICP (Instantaneous Capture Point) planner for each robot.
 * The ICP planner is a module that predicts the desired ICP trajectory accordingly to the upcoming footsteps.
 * This is one of the critical modules of the walking controller as the desired ICP is used to control the balance of the robot.
 *
 * <p>
 * There is two different implementations of the ICP planner, which are referred as the old and new planners.
 * Both can predict the ICP trajectory by considering a constant CMP (Centroidal Momentum Pivot) location when in single support ({@link #useTwoCMPsPerSupport()} == false).
 * In this mode, the constant CMP location used in single support is referred as the entryCMP.
 * <p/>
 *
 * <p>
 * When using the new ICP planner ({@link #useNewICPPlanner()} == true), there is the possibility to use a different mode for planning the ICP trajectory.
 * The mode considers for each single support, an initial CMP (referred here as entryCMP) location that is used at the beginning of single support, and a CMP location that is used at the end of single support (referred here as exitCMP).
 * This mode is referred as using two CMPs per support ({@link #useTwoCMPsPerSupport()} == true) and is preferable as it helps the robot moving more towards the walking heading direction.
 * It also helps triggering the toe off earlier, and helps stepping down obstacles.
 * However, it requires to tune some more parameters.
 * </p>
 */
public abstract class CapturePointPlannerParameters
{
   private final double modelScale;

   protected CapturePointPlannerParameters()
   {
      this(1.0);
   }

   protected CapturePointPlannerParameters(double modelScale)
   {
      this.modelScale = modelScale;
   }

   /** FIXME That's a hack which makes the planner slower than the swing foot. Need to get rid of it. */
   @Deprecated
   public double getAdditionalTimeForSingleSupport()
   {
      return 0.0;
   }

   /**
    * How many footsteps the ICP planner will use to build the plan.
    * The more the better, but will increase the number of YoVariables and increase the computation time.
    * The values 3 and 4 seem to be good.
    */
   public int getNumberOfFootstepsToConsider()
   {
      return 3;
   }

   /**
    * Represents in percent of the current double support duration, how much time the transfer will spend before reaching the next entry CMP.
    * The returned value should be between 0.0 and 1.0:
    * <li> 0.0 is equivalent to spend the entire double support on the initial CMP (last entry CMP if using one CMP per support, last exit CMP otherwise), </li>
    * <li> 1.0 is equivalent to spend the entire double support on the next entry CMP. </li>
    * <p> A value close to 0.5 is preferable. </p>
    */
   public double getDoubleSupportSplitFraction()
   {
      return 0.5;
   }

   /**
    * Slow down factor on the time when doing partial time freeze
    */
   public double getFreezeTimeFactor()
   {
      return 0.9;
   }

   /**
    * Threshold on the ICP error used to trigger the complete time freeze.
    */
   public double getMaxInstantaneousCapturePointErrorForStartingSwing()
   {
      return modelScale * 0.025;
   }

   /**
    * Threshold on the ICP error used to trigger the partial time freeze which consists in slowing down time.
    */
   public double getMaxAllowedErrorWithoutPartialTimeFreeze()
   {
      return modelScale * 0.03;
   }

   /**
    * Enable / disable time freezing.
    * When enabled, the time freezer will first slow down time when the actual ICP is behind the plan, and totally freeze time if the actual ICP is far behing the plan.
    * In summary, this makes the plan wait for the actual ICP when there is a lag.
    */
   public boolean getDoTimeFreezing()
   {
      return false;
   }

   /**
    * This parameter forces the entry CMPs to be more inside or outside in the foot.
    */
   public abstract double getEntryCMPInsideOffset();

   /**
    * Only used when using the new ICP planner with two CMPs per support.
    * This parameter forces the exit CMPs to be more inside or outside in the foot.
    */
   public abstract double getExitCMPInsideOffset();

   /**
    * This parameter forces the entry CMPs to be more forward or backward in the foot.
    */
   public abstract double getEntryCMPForwardOffset();

   /**
    * Only used when using the new ICP planner with two CMPs per support.
    * This parameter forces the exit CMPs to be more forward or backward in the foot.
    */
   public abstract double getExitCMPForwardOffset();

   /**
    * Only used when using the new ICP planner.
    * Refers to whether use one or two CMPs reference locations per support.
    * Using two CMPs is preferable as it helps the robot moving more towards the walking heading direction.
    * It also helps triggering the toe off earlier, and helps stepping down obstacles.
    * However, it requires to tune some more parameters.
    */
   public abstract boolean useTwoCMPsPerSupport();

   /**
    * Only used when using the new ICP planner with two CMPs per support.
    * The returned value should be between 0.0 and 1.0:
    * <li> 0.0 is equivalent to never use the exit CMP, </li>
    * <li> 1.0 is equivalent to never use the entry CMP. </li>
    * <p> A value close to 0.5 is preferable. </p>
    */
   public double getTimeSpentOnExitCMPInPercentOfStepTime()
   {
      return 0.5;
   }

   /**
    * This parameter indicates how far front in the foot the entry CMP can be.
    * A large positive value here can be helpful for back steps.
    */
   public abstract double getMaxEntryCMPForwardOffset();

   /**
    * This parameter indicates how far back in the foot the exit CMP can be.
    * Should probably be 0.0, a negative value would delay the toe off when walking forward.
    */
   public abstract double getMinEntryCMPForwardOffset();

   /**
    * This parameter indicates how far front in the foot the exit CMP can be.
    * A large positive value here can improve significantly long forward steps and help trigger the toe off earlier.
    */
   public abstract double getMaxExitCMPForwardOffset();

   /**
    * This parameter indicates how far back in the foot the exit CMP can be.
    * For instance, -0.02m will let the robot move slightly backward in single support when doing back steps.
    */
   public abstract double getMinExitCMPForwardOffset();

   /**
    * This parameter is used when computing the entry CMP and exit CMP to make sure they are contained inside a safe support polygon.
    */
   public double getCMPSafeDistanceAwayFromSupportEdges()
   {
      return modelScale * 0.01;
   }

   /**
    *  Pretty much refers to how fast the CMP should move from the entry CMP to the exit CMP in single support. 0.5sec seems reasonable.
    *  This parameter allows to reduce unexpected behaviors due to smoothing the exponentially unstable motion of the ICP with a minimum acceleration trajectory (cubic).
    */
   public double getMaxDurationForSmoothingEntryToExitCMPSwitch()
   {
      return 0.5;
   }

   /**
    * Only used when using the new ICP planner with two CMPs per support.
    * The forward offset of the CMPs is computed according to the upcoming step length times the returned factor
    * One third seems to be a reasonable value.
    */
   public double getStepLengthToCMPOffsetFactor()
   {
      return 1.0 / 3.0;
   }

   /**
    * Only used when using the new ICP planner with two CMPs per support.
    * If true, the ICP planner will put the exit CMP on the toes of the trailing foot when stepping down and forward.
    */
   public boolean useExitCMPOnToesForSteppingDown()
   {
      return false;
   }

   /**
    * Only used when using the new ICP planner with two CMPs per support.
    * Threshold used to figure out if the exit CMP should be put on the toes when stepping down.
    */
   public double getStepLengthThresholdForExitCMPOnToesWhenSteppingDown()
   {
      return modelScale * 0.15;
   }

   /**
    * Only used when using the new ICP planner with two CMPs per support.
    * Threshold used to figure out if the exit CMP should be put on the toes.
    * An absolute value is expected.
    */
   public double getStepHeightThresholdForExitCMPOnToesWhenSteppingDown()
   {
      return modelScale * 0.10;
   }

   /**
    * Only used when using the new ICP planner with two CMPs per support.
    * If set to zero, the exit CMP will be on the toes' edge when stepping, a positive value will pull back the exit CMP towards the foot center.
    */
   public double getCMPSafeDistanceAwayFromToesWhenSteppingDown()
   {
      return modelScale * 0.0;
   }

   /**
    * For doing toe off in single support. Set it to 0.0 if not using this feature.
    * @return
    */
   public double getMinTimeToSpendOnExitCMPInSingleSupport()
   {
      return 0.0;
   }

   /**
    * When the plan is done for the current state, the desired ICP velocity then linearly decays to reach 0.0 in the given duration.
    * This is particularly useful when the robot gets stuck in transfer state because the ICP error is too large to switch to swing.
    * Even in that state, the ICP planner is still giving a desired ICP velocity preventing the ICP convergence to a certain extent.
    * This parameter allows to cancel out this desired velocity when stuck in the transfer state helping the convergence of the ICP and will help to get the robot to switch to swing.
    * Set to {@link Double#NaN} or {@link Double#POSITIVE_INFINITY} to not use this feature.
    * A value around 0.5sec to 1.0sec seems reasonable.
    */
   public double getVelocityDecayDurationWhenDone()
   {
      return Double.NaN;
   }

   /**
    * Sets the exit CMP on the toes. If doing toe-off in single support, this is necessary.
    * @return
    */
   public boolean putExitCMPOnToes()
   {
      return false;
   }

   /**
    * Only used when using the new ICP planner with two CMPs per support.
    * Threshold used to figure out if the exit CMP should be put on the toes.
    */
   public double getStepLengthThresholdForExitCMPOnToes()
   {
      return modelScale * 0.15;
   }
}
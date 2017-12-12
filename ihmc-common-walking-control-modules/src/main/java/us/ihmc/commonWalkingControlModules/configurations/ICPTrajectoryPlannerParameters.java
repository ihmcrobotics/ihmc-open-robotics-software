package us.ihmc.commonWalkingControlModules.configurations;

/**
 * Parameters to tune the ICP (Instantaneous Capture Point) planner for each robot. The ICP planner
 * is a module that predicts the desired ICP trajectory accordingly to the upcoming footsteps. This
 * is one of the critical modules of the walking controller as the desired ICP is used to control
 * the balance of the robot.
 *
 * <p>
 * There is two different implementations of the ICP planner, which are referred as the old and new
 * planners. Both can predict the ICP trajectory by considering a constant CMP (Centroidal Momentum
 * Pivot) location when in single support ({@link #useTwoCMPsPerSupport()} == false). In this mode,
 * the constant CMP location used in single support is referred as the entryCMP.
 * <p/>
 *
 * <p>
 * When using the new ICP planner ({@link #useNewICPPlanner()} == true), there is the possibility to
 * use a different mode for planning the ICP trajectory. The mode considers for each single support,
 * an initial CMP (referred here as entryCMP) location that is used at the beginning of single
 * support, and a CMP location that is used at the end of single support (referred here as exitCMP).
 * This mode is referred as using two CMPs per support ({@link #useTwoCMPsPerSupport()} == true) and
 * is preferable as it helps the robot moving more towards the walking heading direction. It also
 * helps triggering the toe off earlier, and helps stepping down obstacles. However, it requires to
 * tune some more parameters.
 * </p>
 */
public interface ICPTrajectoryPlannerParameters
{
   /**
    * How many footsteps the ICP planner will use to build the plan. The more the better, but will
    * increase the number of YoVariables and increase the computation time.
    */
   int getNumberOfFootstepsToConsider();

   double getTransferSplitFraction();

   double getSwingSplitFraction();

   /**
    * <p>
    * Only used when using an ICP planner with two or more CoPs per support.
    * </p>
    * <p>
    * Minimum time spent on the exit CoP in single support. Set to 0.0 if not doing swing toe off.
    * </p>
    * <p>
    * If using two CoPs, determines the minimum amount of time spent on the ball CoP. If using three,
    * determines the amount of time spent on the toe.
    * </p>
    */
   double getMinTimeToSpendOnExitCoPInSingleSupport();

   /**
    * When the plan is done for the current state, the desired ICP velocity then linearly decays to
    * reach 0.0 in the given duration. This is particularly useful when the robot gets stuck in
    * transfer state because the ICP error is too large to switch to swing. Even in that state, the
    * ICP planner is still giving a desired ICP velocity preventing the ICP convergence to a certain
    * extent. This parameter allows to cancel out this desired velocity when stuck in the transfer
    * state helping the convergence of the ICP and will help to get the robot to switch to swing.
    * Set to {@link Double#NaN} or {@link Double#POSITIVE_INFINITY} to not use this feature. A value
    * around 0.5sec to 1.0sec seems reasonable.
    */
   double getVelocityDecayDurationWhenDone();
}
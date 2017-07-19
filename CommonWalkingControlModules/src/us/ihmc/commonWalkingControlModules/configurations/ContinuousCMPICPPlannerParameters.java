package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.List;

/**
 * Parameters to tune the ICP (Instantaneous Capture Point) planner for each robot. The ICP planner
 * is a module that predicts the desired ICP trajectory accordingly to the upcoming footsteps. This
 * is one of the critical modules of the walking controller as the desired ICP is used to control
 * the balance of the robot.
 *
 * <p>
 * There is two different implementations of the ICP planner, which are referred as the old and new
 * planners. Both can predict the ICP trajectory by considering a constant CMP (Centroidal Momentum
 * Pivot) location when in single support ({@link #useTwoCoPsPerSupport()} == false). In this mode,
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
public abstract class ContinuousCMPICPPlannerParameters extends ICPWithTimeFreezingPlannerParameters
{
   private final double modelScale;

   protected ContinuousCMPICPPlannerParameters()
   {
      this(1.0);
   }

   protected ContinuousCMPICPPlannerParameters(double modelScale)
   {
      this.modelScale = modelScale;
   }

   @Override
   /** {@inheritDoc} */
   public boolean useSmoothCMPPlanner()
   {
      return false;
   }

   /**
    * <p>
    * {@inheritDoc}
    * </p>
    * <p>
    * The values 3 and 4 seem to be good.
    * </p>
    */
   @Override
   public int getNumberOfFootstepsToConsider()
   {
      return 3;
   }

   /** {@inheritDoc}
    * <p>
    * This only differentiates between one and two CoPs per foot.
    * </p>
    */
   @Override
   public abstract int getNumberOfCoPWayPointsPerFoot();

   @Override
   /** {@inheritDoc} */
   public double getFreezeTimeFactor()
   {
      return 0.9;
   }

   @Override
   /** {@inheritDoc} */
   public double getMaxInstantaneousCapturePointErrorForStartingSwing()
   {
      return modelScale * 0.025;
   }

   @Override
   /** {@inheritDoc} */
   public double getMaxAllowedErrorWithoutPartialTimeFreeze()
   {
      return modelScale * 0.03;
   }

   @Override
   /** {@inheritDoc} */
   public boolean getDoTimeFreezing()
   {
      return false;
   }




   @Override
   /**
    * Represents in percent the repartition of the double support duration between the previous and
    * next CMP.
    * <p>
    * The output the ICP planner is quite sensitive to this parameter and the plan
    * quality/feasibility can be greatly reduced with bad a tuning.
    * </p>
    */
   public double getTransferSplitFraction()
   {
      return 0.5;
   }


   @Override
   /**
    * Represents in percent the repartition of the single support duration between the entry and
    * exit CMP.
    * <p>
    * Only used when {@link #useTwoCoPsPerSupport()} is {@code true}.
    * </p>
    * <p>
    * The output the ICP planner is quite sensitive to this parameter and the plan
    * quality/feasibility can be greatly reduced with bad a tuning.
    * </p>
    */
   public double getSwingSplitFraction()
   {
      return 0.5;
   }

   @Override
   /** {@inheritDoc} */
   public double getMinTimeToSpendOnExitCoPInSingleSupport()
   {
      return 0.0;
   }

   @Override
   /** {@inheritDoc} */
   public double getVelocityDecayDurationWhenDone()
   {
      return Double.NaN;
   }





   /**
    * <p>
    * {@inheritDoc}
    * </p>
    * <p>
    * A large positive maximum value for the ball CoP here can improve significantly long
    * forward steps and help trigger the toe off earlier.
    * The minimum value for the ball CoP indicates how far back in the foot it can be. For instance,
    * -0.02m will let the robot move slightly backward in single support when doing back steps.
    * </p>
    * <p>
    * The data is organized such that the heel CoP bounds are the first entry in the list, and
    * the ball CoP bounds are the second entry in the list. The minimum value is then the X field
    * in the vector, while the maximum value is the Y field in the vector.
    * </p>
    */
   @Override
   public abstract List<Vector2D> getCoPForwardOffsetBounds();

   /**
    * <p>
    * {@inheritDoc}
    * </p>
    * <p>
    * The first entry contains the offsets for the entry CoP, while the second entry contains
    * the offsets for the exit CoP. The values for the second CoP are only used when using an
    * ICP planner with two or more CoPs per support.
    * </p>
    * <p>
    * The offsets themselves are in the foot frame. The X offset is the forward offset in the foot,
    * while the Y offset is the inside offset.
    * </p>
    */
   @Override
   public abstract List<Vector2D> getCoPOffsets();


   @Override
   /** {@inheritDoc} */
   public double getCoPSafeDistanceAwayFromSupportEdges()
   {
      return modelScale * 0.01;
   }

   @Override
   /** {@inheritDoc} */
   public double getStepLengthToBallCoPOffsetFactor()
   {
      return 1.0 / 3.0;
   }

   @Override
   /** {@inheritDoc} */
   public boolean putExitCoPOnToes()
   {
      return false;
   }


   @Override
   /** {@inheritDoc} */
   public boolean useExitCoPOnToesForSteppingDown()
   {
      return false;
   }

   @Override
   /** {@inheritDoc} */
   public double getStepLengthThresholdForExitCoPOnToesWhenSteppingDown()
   {
      return modelScale * 0.15;
   }

   @Override
   /** {@inheritDoc} */
   public double getStepHeightThresholdForExitCoPOnToesWhenSteppingDown()
   {
      return modelScale * 0.10;
   }

   @Override
   /** {@inheritDoc} */
   public double getCoPSafeDistanceAwayFromToesWhenSteppingDown()
   {
      return modelScale * 0.0;
   }

   @Override
   /** {@inheritDoc} */
   public double getExitCoPForwardSafetyMarginOnToes()
   {
      return modelScale * 1.6e-2;
   }

   @Override
   /** {@inheritDoc} */
   public double getStepLengthThresholdForExitCoPOnToes()
   {
      return modelScale * 0.15;
   }

   @Override
   /** {@inheritDoc} */
   public double getMaxDurationForSmoothingEntryToExitCoPSwitch()
   {
      return 0.5;
   }
}
package us.ihmc.commonWalkingControlModules.configurations;

import java.util.EnumMap;

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
 * This mode is referred as using two CMPs per support ({@link #getNumberOfCoPWayPointsPerFoot()} == 2) and
 * is preferable as it helps the robot moving more towards the walking heading direction. It also
 * helps triggering the toe off earlier, and helps stepping down obstacles. However, it requires to
 * tune some more parameters.
 * </p>
 */
public abstract class ContinuousCMPICPPlannerParameters extends AbstractICPPlannerParameters
{
   /**
    * Ordered list of fractions indicating whether CoP offset changes with step length
    */
   protected final EnumMap<CoPPointName, Double> stepLengthToCoPOffsetFactor = new EnumMap<>(CoPPointName.class);

   /** Indicate the first CoP for the swing phase */
   protected CoPPointName entryCoPName = CoPPointName.HEEL_COP;
   /** Indicate the last CoP for the swing phase. Typically everything for this point should be determined from the final values otherwise computation is not possible */
   protected CoPPointName exitCoPName = CoPPointName.TOE_COP;

   protected ContinuousCMPICPPlannerParameters()
   {
      this(1.0);
   }

   protected ContinuousCMPICPPlannerParameters(double modelScale)
   {
      super(modelScale);

      stepLengthToCoPOffsetFactor.put(entryCoPName, 0.0);
      stepLengthToCoPOffsetFactor.put(exitCoPName, 1.0 / 3.0);
   }

   @Override
   /** {@inheritDoc} */
   public boolean useSmoothCMPPlanner()
   {
      return false;
   }

   /**
    * Represents in percent the repartition of the double support duration between the previous and
    * next CMP.
    * <p>
    * The output the ICP planner is quite sensitive to this parameter and the plan
    * quality/feasibility can be greatly reduced with bad a tuning.
    * </p>
    */
   @Override
   public double getTransferSplitFraction()
   {
      return 0.5;
   }


   /**
    * Represents in percent the repartition of the single support duration between the entry and
    * exit CMP.
    * <p>
    * Only used when {@link #getNumberOfCoPWayPointsPerFoot()} is {@code 2}.
    * </p>
    * <p>
    * The output the ICP planner is quite sensitive to this parameter and the plan
    * quality/feasibility can be greatly reduced with bad a tuning.
    * </p>
    */
   @Override
   public double getSwingSplitFraction()
   {
      return 0.5;
   }

   @Override
   /** {@inheritDoc} */
   public EnumMap<CoPPointName, Double> getStepLengthToCoPOffsetFactors()
   {
      stepLengthToCoPOffsetFactor.put(entryCoPName, 0.0);
      stepLengthToCoPOffsetFactor.put(exitCoPName, 1.0 / 3.0);
      return stepLengthToCoPOffsetFactor;
   }

   /**
    * <p>
    * Pretty much refers to how fast the CoP should move from the entry CoP to the exit CoP in
    * single support. 0.5sec seems reasonable. This parameter allows to reduce unexpected behaviors
    * due to smoothing the exponentially unstable motion of the ICP with a minimum acceleration
    * trajectory (cubic).
    * </p>
    * Note this is only used when using the smooth cmp trajectory generator.
    */
   public double getMaxDurationForSmoothingEntryToExitCoPSwitch()
   {
      return 0.5;
   }
}
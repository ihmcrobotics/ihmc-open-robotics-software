package us.ihmc.commonWalkingControlModules.configurations;

import java.util.EnumMap;

import us.ihmc.euclid.tuple2D.Vector2D;

public class SmoothCMPPlannerParameters implements ICPWithTimeFreezingPlannerParameters
{
   private static final boolean adjustPlanForSingleSupport = false;
   private static final boolean adjustPlanForInitialDoubleSupport = true;
   private static final boolean adjustPlanForEachDoubleSupport = true;
   private static final boolean adjustPlanWhenGoingToStand = true;

   private static final boolean doContinuousReplanningForStanding = false;
   private static final boolean doContinuousReplanningForTransfer = false;
   private static final boolean doContinuousReplanningForSwing = false;

   /**
    * Vector offsets relative to centroid of support polygon defined copOffsetFrames
    */
   protected final EnumMap<CoPPointName, Vector2D> copOffsetsInFootFrame = new EnumMap<>(CoPPointName.class);
   /**
    * Bounds of the CoP offsets in the foot frame
    */
   protected final EnumMap<CoPPointName, Vector2D> copOffsetBoundsInFootFrame = new EnumMap<>(CoPPointName.class);

   /**
    * Ordered list of fractions indicating whether CoP offset changes with step length
    */
   protected final EnumMap<CoPPointName, Double> stepLengthToCoPOffsetFactor = new EnumMap<>(CoPPointName.class);

   /**
    * Last transfer is planned till the endCoP in the transferCoPPointsToPlan
    */
   protected CoPPointName[] swingCopPointsToPlan;
   protected CoPPointName[] transferCoPPointsToPlan;

   /** Final CoP name (chicken support will be used only for this point). In case */
   protected CoPPointName endCoPName;
   /** Indicate the first CoP for the swing phase */
   protected CoPPointName entryCoPName;
   /** Indicate the last CoP for the swing phase. Typically everything for this point should be determined from the final values otherwise computation is not possible */
   protected CoPPointName exitCoPName;

   protected final double modelScale;

   public SmoothCMPPlannerParameters()
   {
      this(1.0);
   }

   public SmoothCMPPlannerParameters(double modelScale)
   {
      this.modelScale = modelScale;

      this.swingCopPointsToPlan = new CoPPointName[] {CoPPointName.MIDFOOT_COP, CoPPointName.EXIT_COP};
      this.transferCoPPointsToPlan = new CoPPointName[] {CoPPointName.MIDFEET_COP, CoPPointName.ENTRY_COP};
      this.exitCoPName = CoPPointName.EXIT_COP;
      this.entryCoPName = CoPPointName.ENTRY_COP;
      this.endCoPName = CoPPointName.MIDFEET_COP;

      stepLengthToCoPOffsetFactor.put(CoPPointName.MIDFEET_COP, 0.0);
      stepLengthToCoPOffsetFactor.put(CoPPointName.ENTRY_COP, 1.0 / 3.0);
      stepLengthToCoPOffsetFactor.put(CoPPointName.MIDFOOT_COP, 1.0 / 8.0);
      stepLengthToCoPOffsetFactor.put(CoPPointName.EXIT_COP, 1.0 / 3.0);

      copOffsetsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(0.0, 0.0));
      copOffsetsInFootFrame.put(CoPPointName.ENTRY_COP, new Vector2D(0.0, -0.005));
      copOffsetsInFootFrame.put(CoPPointName.MIDFOOT_COP, new Vector2D(0.0, 0.01));
      copOffsetsInFootFrame.put(CoPPointName.EXIT_COP, new Vector2D(0.0, 0.025));

      copOffsetBoundsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY));
      copOffsetBoundsInFootFrame.put(CoPPointName.ENTRY_COP, new Vector2D(-0.04, 0.03));
      copOffsetBoundsInFootFrame.put(CoPPointName.MIDFOOT_COP, new Vector2D(0.0, 0.055));
      copOffsetBoundsInFootFrame.put(CoPPointName.EXIT_COP, new Vector2D(0.0, 0.08));
   }

   @Override
   public boolean planSwingAngularMomentum()
   {
      return false;
   }

   @Override
   public boolean planTransferAngularMomentum()
   {
      return false;
   }

   @Override
   /** {@inheritDoc} */
   public int getNumberOfCoPWayPointsPerFoot()
   {
      return swingCopPointsToPlan.length + 1;
   }

   /**
    * The transfer phase is split into two halves. This introduces a midpoint between
    * the initial and final CoP, which represents how far to shift the CoP during the first phase.
    */
   @Override
   public double getTransferSplitFraction()
   {
      return 0.5;
   }

   /**
    * <p>
    * Only used when using an ICP planner with two or more CoP waypoints.
    * </p>
    * <p>
    * If there are two CoP waypoints, introduces a midpoint between the initial and final CoP,
    * which represents how far to shift the CoP during the first phase.
    * </p>
    * <p>
    * If there are three CoP waypoints, represents the fraction of the swing duration spent
    * shifting to the first waypoint.
    * </p>
    */
   @Override
   public double getSwingSplitFraction()
   {
      return 0.5;
   }

   /**
    * Fraction of the swing duration which the CoP is spent shifting the CoP from the heel CoP to the exit CoP.
    * The remaining fraction of swing is spent on the exit CoP.
    * The value calculated using this must be greater than that returned by {@link #getMinTimeToSpendOnExitCoPInSingleSupport()},
    * or it is overwritten.
    */
   @Override
   public double getSwingDurationShiftFraction()
   {
      return 0.90;
   }

   /** {@inheritDoc} */
   @Override
   public EnumMap<CoPPointName, Vector2D> getCoPForwardOffsetBoundsInFoot()
   {
      return copOffsetBoundsInFootFrame;
   }

   /** {@inheritDoc} */
   @Override
   public EnumMap<CoPPointName, Vector2D> getCoPOffsetsInFootFrame()
   {
      return copOffsetsInFootFrame;
   }

   @Override
   public CoPPointName getExitCoPName()
   {
      return exitCoPName;
   }

   @Override
   public CoPPointName[] getSwingCoPPointsToPlan()
   {
      return swingCopPointsToPlan;
   }

   @Override
   public CoPPointName[] getTransferCoPPointsToPlan()
   {
      return transferCoPPointsToPlan;
   }

   @Override
   /** {@inheritDoc} */
   public EnumMap<CoPPointName, Double> getStepLengthToCoPOffsetFactors()
   {
      return stepLengthToCoPOffsetFactor;
   }

   @Override
   public CoPSplineType getOrderOfCoPInterpolation()
   {
      return CoPSplineType.LINEAR;
   }

   @Override
   public AngularMomentumEstimationParameters getAngularMomentumEstimationParameters()
   {
      return new AngularMomentumEstimationParameters();
   }

   @Override
   public boolean adjustCoPPlanForSingleSupportContinuity()
   {
      return adjustPlanForSingleSupport;
   }

   @Override
   public boolean adjustInitialCoPPlanForDoubleSupportContinuity()
   {
      return adjustPlanForInitialDoubleSupport;
   }

   @Override
   public boolean adjustEveryCoPPlanForDoubleSupportContinuity()
   {
      return adjustPlanForEachDoubleSupport;
   }

   @Override
   public boolean adjustCoPPlanForStandingContinuity()
   {
      return adjustPlanWhenGoingToStand;
   }

   @Override
   public boolean doContinuousReplanningForStanding()
   {
      return doContinuousReplanningForStanding;
   }

   @Override
   public boolean doContinuousReplanningForTransfer()
   {
      return doContinuousReplanningForTransfer;
   }

   @Override
   public boolean doContinuousReplanningForSwing()
   {
      return doContinuousReplanningForSwing;
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

   @Override
   public double getMaxInstantaneousCapturePointErrorForStartingSwing()
   {
      return modelScale * 0.025;
   }

   @Override
   public double getMaxAllowedErrorWithoutPartialTimeFreeze()
   {
      return modelScale * 0.03;
   }

   @Override
   public boolean getDoTimeFreezing()
   {
      return false;
   }

   @Override
   public double getMinTimeToSpendOnExitCoPInSingleSupport()
   {
      return 0.0;
   }

   @Override
   public double getVelocityDecayDurationWhenDone()
   {
      return Double.NaN;
   }

   @Override
   public double getCoPSafeDistanceAwayFromSupportEdges()
   {
      return modelScale * 0.01;
   }

   @Override
   public boolean putExitCoPOnToes()
   {
      return false;
   }

   @Override
   public boolean useExitCoPOnToesForSteppingDown()
   {
      return false;
   }

   @Override
   public double getStepLengthThresholdForExitCoPOnToesWhenSteppingDown()
   {
      return modelScale * 0.15;
   }

   @Override
   public double getStepHeightThresholdForExitCoPOnToesWhenSteppingDown()
   {
      return modelScale * 0.10;
   }

   @Override
   public double getCoPSafeDistanceAwayFromToesWhenSteppingDown()
   {
      return modelScale * 0.0;
   }

   @Override
   public double getExitCoPForwardSafetyMarginOnToes()
   {
      return modelScale * 1.6e-2;
   }

   @Override
   public double getStepLengthThresholdForExitCoPOnToes()
   {
      return modelScale * 0.15;
   }
}

package us.ihmc.commonWalkingControlModules.configurations;

import java.util.EnumMap;

import us.ihmc.euclid.tuple2D.Vector2D;

public class SmoothCMPPlannerParameters extends AbstractICPPlannerParameters
{
   /**
    * Vector offsets relative to centroid of support polygon defined copOffsetFrames
    */
   protected final EnumMap<CoPPointName, Vector2D> copOffsetsInFootFrame = new EnumMap<>(CoPPointName.class);
   /**
    * Bounds of the CoP offsets in the foot frame
    */
   protected final EnumMap<CoPPointName, Vector2D> copOffsetBoundsInFootFrame = new EnumMap<>(CoPPointName.class);
   protected final EnumMap<CoPPointName, CoPSupportPolygonNames> copOffsetFrameNames = new EnumMap<>(CoPPointName.class);

   /**
    * Order list of flags indicating whether specified bounding boxes should be used to constrain the CoP point
    */
   protected final EnumMap<CoPPointName, Boolean> constrainToMinMax = new EnumMap<>(CoPPointName.class);
   /**
    * Order list of flags indicating whether CoP should reside within the support polygon specified in copOffsetFrames
    */
   protected final EnumMap<CoPPointName, Boolean> constrainToSupportPolygon = new EnumMap<>(CoPPointName.class);
   protected final EnumMap<CoPPointName, CoPSupportPolygonNames> stepLengthOffsetPolygon = new EnumMap<>(CoPPointName.class);
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

   public SmoothCMPPlannerParameters()
   {
      this(1.0);
   }

   public SmoothCMPPlannerParameters(double modelScale)
   {
      super(modelScale);

      this.swingCopPointsToPlan = new CoPPointName[] {CoPPointName.BALL_COP, CoPPointName.TOE_COP};
      this.transferCoPPointsToPlan = new CoPPointName[] {CoPPointName.MIDFEET_COP, CoPPointName.HEEL_COP};
      this.exitCoPName = CoPPointName.TOE_COP;
      this.entryCoPName = CoPPointName.HEEL_COP;
      this.endCoPName = CoPPointName.MIDFEET_COP;
      
      copOffsetFrameNames.put(CoPPointName.HEEL_COP, CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON);
      copOffsetFrameNames.put(CoPPointName.BALL_COP, CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON);
      copOffsetFrameNames.put(CoPPointName.TOE_COP, CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON);
      copOffsetFrameNames.put(CoPPointName.MIDFEET_COP, CoPSupportPolygonNames.INITIAL_DOUBLE_SUPPORT_POLYGON);

      stepLengthOffsetPolygon.put(CoPPointName.MIDFEET_COP, CoPSupportPolygonNames.NULL);
      stepLengthOffsetPolygon.put(CoPPointName.HEEL_COP, CoPSupportPolygonNames.INITIAL_SWING_POLYGON);
      stepLengthOffsetPolygon.put(CoPPointName.BALL_COP, CoPSupportPolygonNames.FINAL_SWING_POLYGON);
      stepLengthOffsetPolygon.put(CoPPointName.TOE_COP, CoPSupportPolygonNames.FINAL_SWING_POLYGON);

      constrainToMinMax.put(CoPPointName.MIDFEET_COP, false);
      constrainToMinMax.put(CoPPointName.HEEL_COP, true);
      constrainToMinMax.put(CoPPointName.BALL_COP, true);
      constrainToMinMax.put(CoPPointName.TOE_COP, true);

      constrainToSupportPolygon.put(CoPPointName.MIDFEET_COP, false);
      constrainToSupportPolygon.put(CoPPointName.HEEL_COP, false);
      constrainToSupportPolygon.put(CoPPointName.BALL_COP, false);
      constrainToSupportPolygon.put(CoPPointName.TOE_COP, false);

      stepLengthToCoPOffsetFactor.put(CoPPointName.MIDFEET_COP, 0.0);
      stepLengthToCoPOffsetFactor.put(CoPPointName.HEEL_COP, 1.0 / 3.0);
      stepLengthToCoPOffsetFactor.put(CoPPointName.BALL_COP, 1.0 / 8.0);
      stepLengthToCoPOffsetFactor.put(CoPPointName.TOE_COP, 1.0 / 3.0);

      copOffsetsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(0.0, 0.0));
      copOffsetsInFootFrame.put(CoPPointName.HEEL_COP, new Vector2D(0.0, -0.005));
      copOffsetsInFootFrame.put(CoPPointName.BALL_COP, new Vector2D(0.0, 0.01));
      copOffsetsInFootFrame.put(CoPPointName.TOE_COP, new Vector2D(0.0, 0.025));

      copOffsetBoundsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY));
      copOffsetBoundsInFootFrame.put(CoPPointName.HEEL_COP, new Vector2D(-0.04, 0.03));
      copOffsetBoundsInFootFrame.put(CoPPointName.BALL_COP, new Vector2D(0.0, 0.055));
      copOffsetBoundsInFootFrame.put(CoPPointName.TOE_COP, new Vector2D(0.0, 0.08));
   }

   public boolean planWithAngularMomentum()
   {
      return false;
   }

   @Override
   /** {@inheritDoc} */
   public boolean useSmoothCMPPlanner()
   {
      return true;
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

   public EnumMap<CoPPointName, CoPSupportPolygonNames> getSupportPolygonNames()
   {
      return copOffsetFrameNames;
   }

   @Override
   public CoPPointName getEntryCoPName()
   {
      return entryCoPName;
   }

   @Override
   public CoPPointName getExitCoPName()
   {
      return exitCoPName;
   }

   public CoPPointName getEndCoPName()
   {
      return endCoPName;
   }

   public CoPPointName[] getSwingCoPPointsToPlan()
   {
      return swingCopPointsToPlan;
   }

   public CoPPointName[] getTransferCoPPointsToPlan()
   {
      return transferCoPPointsToPlan;
   }

   public EnumMap<CoPPointName, Boolean> getIsConstrainedToMinMaxFlags()
   {
      return constrainToMinMax;
   }

   public EnumMap<CoPPointName, Boolean> getIsConstrainedToSupportPolygonFlags()
   {
      return constrainToSupportPolygon;
   }

   public EnumMap<CoPPointName, CoPSupportPolygonNames> getStepLengthCoPOffsetPolygons()
   {
      return stepLengthOffsetPolygon;
   }

   @Override
   /** {@inheritDoc} */
   public EnumMap<CoPPointName, Double> getStepLengthToCoPOffsetFactors()
   {
      return stepLengthToCoPOffsetFactor;
   }

   public CoPSplineType getOrderOfCoPInterpolation()
   {
      return CoPSplineType.LINEAR;
   }

   public AngularMomentumEstimationParameters getAngularMomentumEstimationParameters()
   {
      return new AngularMomentumEstimationParameters();
   }
}

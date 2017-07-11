package us.ihmc.commonWalkingControlModules.configurations;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class SmoothCMPPlannerParameters extends ICPPlannerParameters
{
   private final double modelScale;
   private final List<Vector2D> copOffsetsFootFrame = new ArrayList<>();
   private final EnumMap<CoPPointName, Double> maxXCoPOffsets = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, Double> minXCoPOffsets = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, Double> maxYCoPOffsets = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, Double> minYCoPOffsets = new EnumMap<>(CoPPointName.class);

   /**
    * List of CoP points to plan in the order of planning
    */
   private final CoPPointName[] copPointsToPlan = {CoPPointName.HEEL_COP, CoPPointName.BALL_COP};
   /**
    * CoP offsets in foot frame
    */
   private final Vector2D[] copOffsets = {new Vector2D(0.0, -0.005), new Vector2D(0.0, 0.025)};
   private final BoundingBox2D[] copOffsetLimits = {new BoundingBox2D(-0.04, -1, 0.03, 1), new BoundingBox2D(0.0, -1, 0.08, -1)};
   /**
    * Final CoP name (chicken support will be used only for this point)
    */
   private final CoPPointName endCoP = CoPPointName.MIDFEET_COP;
   /**
    * Percentage chicken support
    */
   private final double chickenSuppportPercentage = 0.5;

   public SmoothCMPPlannerParameters()
   {
      this(1.0);
   }

   public SmoothCMPPlannerParameters(double modelScale)
   {
      this.modelScale = modelScale;
   }

   @Override
   /** {@inheritDoc} */
   public boolean useSmoothCMPPlanner()
   {
      return true;
   }
   @Override
   /** {@inheritDoc} */
   public int getNumberOfFootstepsToConsider()
   {
      return 3;
   }

   @Override
   /** {@inheritDoc} */
   public int getNumberOfCoPWayPointsPerFoot()
   {
      return copPointsToPlan.length;
   }

   @Override
   /**
    * The transfer phase is split into two halves. This introduces a midpoint between
    * the initial and final CoP, which represents how far to shift the CoP during the first phase.
    */
   public double getTransferSplitFraction()
   {
      return 0.5;
   }

   @Override
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
      return 0.8;
   }

   @Override
   /** {@inheritDoc} */
   public double getMinTimeToSpendOnExitCoPInSingleSupport()
   {
      return 0.0;
   }

   public EnumMap<CoPPointName, Double> getMaxXCoPOffsets()
   {
      for (int i = 0; i < copPointsToPlan.length; i++)
      {
         maxXCoPOffsets.put(copPointsToPlan[i], copOffsetLimits[i].getMaxX() * modelScale);
      }
      return maxXCoPOffsets;
   }

   public EnumMap<CoPPointName, Double> getMinXCoPOffsets()
   {
      for (int i = 0; i < copPointsToPlan.length; i++)
      {
         minXCoPOffsets.put(copPointsToPlan[i], copOffsetLimits[i].getMinX() * modelScale);
      }
      return minXCoPOffsets;
   }

   @Override
   /** {@inheritDoc} */
   public double getVelocityDecayDurationWhenDone()
   {
      return Double.NaN;
   }

   @Override
   /**
    * This is not used in this planner.
    */
   public double getMaxDurationForSmoothingEntryToExitCoPSwitch()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   /**
    * <p>
    *  {@inheritDoc}
    * </p>
    * <p>
    * The data is organized such that the heel CoP bounds are the first entry in the list,
    * the ball CoP bounds are the second entry in the list, and the toe CoP bounds are the
    * third entry in the list. The minimum value is the X field in the vector, while the
    * maximum value is the Y field in the vector.
    * </p>
    */
   public List<Vector2D> getCoPForwardOffsetBounds()
   {
      Vector2D heelBounds = new Vector2D(-0.04, 0.03);
      Vector2D ballBounds = new Vector2D(0.0, 0.08);

      heelBounds.scale(modelScale);
      ballBounds.scale(modelScale);

      List<Vector2D> copBounds = new ArrayList<>();
      copBounds.add(heelBounds);
      copBounds.add(ballBounds);

      return copBounds;
   }

   @Override
   /**
    * <p>
    * {@inheritDoc}
    * </p>
    * <p>
    * The first entry contains the offsets for the heel CoP, the second entry contains the offsets
    * for the ball CoP, and the third entry contains the offsets for the toe second.
    * </p>
    */
   public List<Vector2D> getCoPOffsets()
   {
      Vector2D tempVec;
      for (int i = 0; i < copPointsToPlan.length; i++)
      {
         tempVec = copOffsets[i];
         tempVec.scale(modelScale);
         copOffsetsFootFrame.add(tempVec);
      }
      return copOffsetsFootFrame;
   }
   
   public CoPPointName getEntryCoPName()
   {
      return copPointsToPlan[0];
   }
   
   public CoPPointName getExitCoPName()
   {
      return copPointsToPlan[copPointsToPlan.length -1];
   }

   @Override
   /** {@inheritDoc} */
   public double getCoPSafeDistanceAwayFromSupportEdges()
   {
      return modelScale * 0.01;
   }

   public CoPPointName[] getCoPPointsToPlan()
   {
      return copPointsToPlan;
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
   public double getStepHeightThresholdForExitCoPOnToesWhenSteppingDown()
   {
      return 0;
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

   /**
    * Provides a list of the alphas that denote the percentage of time taken to transition from one CoP way point to another.
    * Summation of the list must be equal to 1. 
    * @return
    */
   public CoPSplineType getOrderOfCoPInterpolation()
   {
      return CoPSplineType.LINEAR;
   }
}

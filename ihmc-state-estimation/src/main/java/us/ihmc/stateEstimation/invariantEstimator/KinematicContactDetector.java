package us.ihmc.stateEstimation.invariantEstimator;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Forward-kinematics-only fallback {@link ContactProbabilityProvider}: estimates per-foot contact
 * probability from sole height above a support plane and sole vertical speed. No force/torque sensing is
 * required, so this is the contact estimator that runs unchanged on Digit (whose foot F/T sensors were
 * removed); it is also exactly the hand-coded binary contact logic that a learned ContactNet covariance
 * is meant to supersede.
 *
 * <p>For each foot the probability is the product of two logistic gates:
 * <pre>
 *   p_height = σ((h0 − h) / wHeight)        h  = sole height above the support plane
 *   p_speed  = σ((v0 − |ż|) / wSpeed)       ż  = sole vertical speed (finite-difference)
 *   p        = smooth( p_height · p_speed )
 * </pre>
 * The height gate is primary (a lifted foot reads low regardless of speed, which also covers swing apex
 * where {@code ż ≈ 0}); the speed gate suppresses the brief near-ground transients at lift-off and
 * touchdown. A light exponential smoother removes single-tick chatter. The output is continuous, which is
 * what the estimator's soft-inflation laws consume.</p>
 *
 * <p>The support plane is a single height {@code groundHeight} (flat ground). For uneven terrain, swap
 * the height reference for an estimated local ground height or a relative lowest-foot scheme; only
 * {@link #soleHeightAboveGround} would change.</p>
 *
 * <p>Not allocation-free on the first tick (one {@link FramePoint3D} per foot is reused thereafter).
 * Reads sole frames only; if constructed with a non-null {@code frameUpdater} it runs that first each
 * tick (used when the sole frames belong to a model this detector must refresh itself, e.g. an
 * independent controller-model oracle; pass null when the caller already updates the frames).</p>
 */
public class KinematicContactDetector implements ContactProbabilityProvider
{
   private final SideDependentList<MovingReferenceFrame> soleFrames;
   private final Runnable frameUpdater; // nullable: refreshes soleFrames' tree when this detector owns it
   private final double dt;

   private final double groundHeight;
   private final double heightThreshold;   // h0: half-transition height (m)
   private final double heightWidth;       // wHeight: logistic width in height (m)
   private final double speedThreshold;    // v0: half-transition vertical speed (m/s)
   private final double speedWidth;        // wSpeed: logistic width in speed (m/s)
   private final double smoothingAlpha;    // exponential smoother in [0, 1); 0 = no smoothing

   private final SideDependentList<FramePoint3D> soleInWorld = new SideDependentList<>();
   // Indexed by RobotSide.ordinal() (LEFT=0, RIGHT=1). Primitive arrays so the per-tick finite-difference
   // and smoother writes cause no Double autoboxing on the estimator hot path.
   private final double[] previousHeight = {Double.NaN, Double.NaN};
   private final double[] probability = {0.0, 0.0};

   /** Builds a detector with reasonable flat-ground defaults; only the sole frames and Δt are required. */
   public KinematicContactDetector(SideDependentList<MovingReferenceFrame> soleFrames, Runnable frameUpdater, double dt)
   {
      // groundHeight=0 (flat), h0=4cm w=8mm (planted≈1, lifted>8cm≈0), v0=0.5m/s w=0.1, light smoothing.
      this(soleFrames, frameUpdater, dt, 0.0, 0.04, 0.008, 0.50, 0.10, 0.6);
   }

   /**
    * @param soleFrames      the per-foot sole frames to read.
    * @param frameUpdater    optional hook run at the start of each {@link #update()} to refresh the sole
    *                        frames' tree; pass {@code null} if the caller already updates them.
    * @param dt              the tick period Δt (s), used to finite-difference sole vertical speed.
    * @param groundHeight    the support-plane height (m) in world.
    * @param heightThreshold h0: sole height above ground at which contact probability crosses 0.5 (m).
    * @param heightWidth     logistic width of the height gate (m).
    * @param speedThreshold  v0: sole vertical speed at which the speed gate crosses 0.5 (m/s).
    * @param speedWidth      logistic width of the speed gate (m/s).
    * @param smoothingAlpha  exponential smoothing factor in [0, 1): p ← α·p_prev + (1−α)·p_raw.
    */
   public KinematicContactDetector(SideDependentList<MovingReferenceFrame> soleFrames,
                                   Runnable frameUpdater,
                                   double dt,
                                   double groundHeight,
                                   double heightThreshold,
                                   double heightWidth,
                                   double speedThreshold,
                                   double speedWidth,
                                   double smoothingAlpha)
   {
      this.soleFrames = soleFrames;
      this.frameUpdater = frameUpdater;
      this.dt = dt;
      this.groundHeight = groundHeight;
      this.heightThreshold = heightThreshold;
      this.heightWidth = heightWidth;
      this.speedThreshold = speedThreshold;
      this.speedWidth = speedWidth;
      this.smoothingAlpha = smoothingAlpha;

      for (RobotSide side : RobotSide.values)
         soleInWorld.put(side, new FramePoint3D());
   }

   @Override
   public void update()
   {
      if (frameUpdater != null)
         frameUpdater.run();

      for (RobotSide side : RobotSide.values)
      {
         double height = soleHeightAboveGround(side);

         double previous = previousHeight[side.ordinal()];
         double verticalSpeed = Double.isNaN(previous) ? 0.0 : (height - previous) / dt;
         previousHeight[side.ordinal()] = height;

         double pHeight = logistic((heightThreshold - height) / heightWidth);
         double pSpeed = logistic((speedThreshold - Math.abs(verticalSpeed)) / speedWidth);
         double pRaw = pHeight * pSpeed;

         double pSmoothed = smoothingAlpha * probability[side.ordinal()] + (1.0 - smoothingAlpha) * pRaw;
         probability[side.ordinal()] = clamp(pSmoothed);
      }
   }

   @Override
   public double getContactProbability(RobotSide side)
   {
      return probability[side.ordinal()];
   }

   /** @return the sole-origin height above the support plane (m), in world. */
   private double soleHeightAboveGround(RobotSide side)
   {
      FramePoint3D soleWorld = soleInWorld.get(side);
      soleWorld.setToZero(soleFrames.get(side));
      soleWorld.changeFrame(ReferenceFrame.getWorldFrame());
      return soleWorld.getZ() - groundHeight;
   }

   private static double logistic(double x)
   {
      return 1.0 / (1.0 + Math.exp(-x));
   }

   private static double clamp(double value)
   {
      return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
   }
}

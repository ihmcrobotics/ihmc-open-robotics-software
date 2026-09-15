package us.ihmc.stateEstimation.invariantEstimator;

/**
 * One-shot-per-contact-cycle arming latch for the touchdown anchor re-seed (H4 Phase 2, see
 * {@link InvariantEKF#reseedContact} and the 2026-07-16 derivation note).
 *
 * <p>The re-seed fires once when the contact probability rises through the trigger threshold while
 * armed, and must re-arm before it can fire again. Re-arming requires the probability to stay below
 * the re-arm threshold for a <em>sustained dwell</em>, not just a single tick: on Alex the
 * joint-torque foot switch's force estimate can collapse mid-strike (CoP-under-ankle, hardware log
 * 20260717_112516), producing a p ≈ 1 → 0 → 1 double pulse within one physical touchdown. A
 * single-tick re-arm turned that into two re-seeds per strike; the dwell requires a genuine swing
 * phase (hundreds of ms at p ≈ 0) before the next re-seed, so one strike fires at most once per
 * sustained-low episode.</p>
 *
 * <p>Plain state machine, no Yo* dependencies, so the arming property is unit-testable in isolation
 * ({@code TouchdownReseedLatchTest}).</p>
 */
public class TouchdownReseedLatch
{
   private final double triggerProbability;
   private final double rearmProbability;
   private final int rearmDwellTicks;

   private boolean armed;
   private int lowDwellCounter = 0;

   /**
    * @param triggerProbability rising through this while armed fires the re-seed.
    * @param rearmProbability   probability must stay below this to accumulate re-arm dwell.
    * @param rearmDwellTicks    consecutive ticks below {@code rearmProbability} required to re-arm (≥ 1).
    * @param initiallyArmed     whether the latch starts armed (false: arm only after a clean swing).
    */
   public TouchdownReseedLatch(double triggerProbability, double rearmProbability, int rearmDwellTicks, boolean initiallyArmed)
   {
      // Explicit finiteness checks, not folded into the ordering check below: NaN >= x and x >= NaN are both
      // false, so "rearmProbability >= triggerProbability" alone would let a NaN threshold silently pass and
      // the latch would then never re-arm (advance() compares contactProbability against it every tick, and
      // NaN comparisons are always false there too).
      if (!Double.isFinite(triggerProbability))
         throw new IllegalArgumentException("triggerProbability must be finite, was " + triggerProbability);
      if (!Double.isFinite(rearmProbability))
         throw new IllegalArgumentException("rearmProbability must be finite, was " + rearmProbability);
      if (rearmProbability >= triggerProbability)
         throw new IllegalArgumentException("re-arm threshold " + rearmProbability + " must be below trigger " + triggerProbability);
      if (rearmDwellTicks < 1)
         throw new IllegalArgumentException("rearmDwellTicks must be >= 1, was " + rearmDwellTicks);
      this.triggerProbability = triggerProbability;
      this.rearmProbability = rearmProbability;
      this.rearmDwellTicks = rearmDwellTicks;
      this.armed = initiallyArmed;
   }

   /**
    * Advances the latch one tick with this tick's contact probability.
    *
    * @return true exactly on the ticks a re-seed should fire.
    */
   public boolean advance(double contactProbability)
   {
      if (contactProbability < rearmProbability)
      {
         if (!armed && ++lowDwellCounter >= rearmDwellTicks)
            armed = true;
         return false;
      }

      lowDwellCounter = 0;
      if (armed && contactProbability >= triggerProbability)
      {
         armed = false;
         return true;
      }
      return false;
   }

   public boolean isArmed()
   {
      return armed;
   }
}

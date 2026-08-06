package us.ihmc.stateEstimation.invariant_estimator;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * {@link ContactProbabilityProvider} backed by the robot's production {@link FootSwitchInterface}s —
 * on Alex the {@code JointTorqueBasedFootSwitch} (ANKLE_Y torque) selected by
 * {@code StateEstimatorParameters#getFootSwitchFactory()}, i.e. the same contact detection the DRC
 * estimator and the walking controller already trust.
 *
 * <p>This replaces the {@link KinematicContactDetector} fallback for isolation testing and as the
 * default when the invariant filter runs as the main estimator. It fixes the fallback's two known
 * failure modes in that mode:
 * <ul>
 *   <li><b>Circularity:</b> the kinematic detector reads sole heights in world through the estimator
 *       model, which the invariant filter itself poses when promoted to main — unobservable world-Z
 *       drift can mute a planted foot and run away. Joint-torque detection is independent of the
 *       base estimate.</li>
 *   <li><b>Initialization:</b> probabilities start at 1 (feet are planted when
 *       {@code initializeEstimator} runs), instead of 0, so the filter is anchored from the first
 *       tick rather than free-integrating through the initial transient.</li>
 * </ul>
 *
 * <h2>Trust debouncing (2026-07-17)</h2>
 *
 * <p>The raw switch booleans are NOT consumed directly: on Alex the joint-torque switch's force
 * estimate collapses when the CoP passes near the ankle (ANKLE_Y torque → 0 under full load), so
 * {@code hasFootHitGroundFiltered()} produces a spurious up–down–up double pulse per foot strike
 * (hardware log 20260717_112516: 45 filtered transitions for ~11 left steps). The old DRC estimator
 * absorbed this with its {@code isFootTrusted} heuristics; this provider ports the two layers of
 * that armor that apply per foot, as a <em>debounced Schmitt trigger</em> on top of the switch:
 * <ul>
 *   <li><b>Load-percentage hysteresis</b> (port of {@code PelvisLinearStateUpdater}'s
 *       {@code forceInPercentOfWeightThresholdToTrustFoot} / {@code ...ToNotTrustFoot}): a foot must
 *       carry ≥ {@value #DEFAULT_LOAD_ENTER_THRESHOLD} of robot weight to <em>become</em> trusted and
 *       stays trusted until it drops below {@value #DEFAULT_LOAD_STAY_THRESHOLD} — an impact spike on
 *       a bouncing foot does not enter the band, a stance foot breathing around one threshold cannot
 *       chatter.</li>
 *   <li><b>Trust dwell</b> (port of {@code delayTimeBeforeTrustingFoot}, strengthened to act on both
 *       edges): the enter/exit condition must hold for {@value #DEFAULT_TRUST_DWELL_SECONDS} s of
 *       consecutive ticks before the trusted state flips. Measured impact blips hold the enter
 *       condition for ≤ 30 ms; genuine load acceptance holds it for the rest of stance.</li>
 * </ul>
 * A switch that cannot compute a load percentage (NaN) degrades to switch-only trust (the load test
 * passes), preserving the pre-2026-07-17 behavior on robots without force estimates.</p>
 *
 * <p>Probability mapping: trusted {@code → 1}, else {@code hasFootHitGroundSensitive() → 0.5} (fast
 * but unconfirmed touchdown — enough for the estimator's soft R-inflation to lean on the foot
 * lightly, below every 0.5 hard gate), else {@code 0}; then a light exponential smoother so the
 * estimator's soft-inflation laws (which exponentiate {@code 1 − p}) see a continuous signal instead
 * of a step. The foot switches publish their own detector YoVariables into the registry they were
 * built with, so contact decisions remain fully inspectable.</p>
 *
 * <p>Requires the model the switches were built on to have current joint torques and frames when
 * {@link #update()} runs; the estimator guarantees this by calling the provider after its frame
 * update, with joints set upstream (by {@code InvariantMainStateEstimator#updateJoints()} when main,
 * by the DRC estimator's joint updater when secondary).</p>
 */
public class FootSwitchContactProbabilityProvider implements ContactProbabilityProvider
{
   /**
    * Live-switchable trust mode ({@code invariantContactTrustMode} YoEnum), for A/B on hardware and in
    * {@code AlexEstimatorLogReplay} without a rebuild. The Schmitt state machine advances every tick in
    * ALL modes, so switching mid-run is seamless (no stale debounce state).
    */
   public enum TrustMode
   {
      /** Debounced Schmitt trigger (2026-07-17): load hysteresis + dwell. Production default. */
      SCHMITT,
      /** Pre-2026-07-17 behavior: raw switch quantizer {0, 0.5, 1} + EMA, no debounce. A/B arm. */
      LEGACY,
      /** Contact detection off: both feet pinned fully trusted (p = 1, no EMA lag). Control arm — DRC-style permanent foothold trust. */
      NONE
   }

   /** Probability when the foot is not trusted but the sensitive (unconfirmed) detector reports contact. */
   private static final double SENSITIVE_ONLY_PROBABILITY = 0.5;
   /** Default exponential smoothing factor; at 1 kHz this is a ~5 ms time constant. */
   private static final double DEFAULT_SMOOTHING_ALPHA = 0.8;
   /** Fraction of robot weight a foot must carry to BECOME trusted (old {@code ...ThresholdToTrustFoot}). */
   private static final double DEFAULT_LOAD_ENTER_THRESHOLD = 0.35;
   /** Fraction of robot weight below which a trusted foot is dropped (old {@code ...ThresholdToNotTrustFoot}). */
   private static final double DEFAULT_LOAD_STAY_THRESHOLD = 0.25;
   /** Time the enter/exit condition must hold before the trusted state flips (old {@code delayTimeBeforeTrustingFoot}, both edges). */
   private static final double DEFAULT_TRUST_DWELL_SECONDS = 0.04;

   private final SideDependentList<? extends FootSwitchInterface> footSwitches;
   private final double smoothingAlpha;
   private final double loadEnterThreshold;
   private final double loadStayThreshold;
   private final int trustDwellTicks;
   private final YoEnum<TrustMode> trustMode;

   // All indexed by RobotSide.ordinal() (LEFT=0, RIGHT=1). Primitive arrays so the per-tick writes
   // cause no autoboxing on the estimator hot path. Trust starts TRUE / probability at 1 (feet
   // planted when initializeEstimator runs).
   private final double[] probability = {1.0, 1.0};
   private final boolean[] trusted = {true, true};
   private final int[] dwellCounter = {0, 0};

   /** Builds the provider with default smoothing, thresholds, and dwell; trust starts true (feet planted at init). */
   public FootSwitchContactProbabilityProvider(SideDependentList<? extends FootSwitchInterface> footSwitches, double estimatorDT, YoRegistry registry)
   {
      this(footSwitches, estimatorDT, registry, DEFAULT_SMOOTHING_ALPHA, DEFAULT_LOAD_ENTER_THRESHOLD, DEFAULT_LOAD_STAY_THRESHOLD, DEFAULT_TRUST_DWELL_SECONDS);
   }

   /**
    * @param footSwitches       the per-foot switches to poll; this provider calls their {@code update()}.
    * @param estimatorDT        the estimator tick period in seconds; converts {@code trustDwellSeconds} to ticks.
    * @param registry           registry for the {@code invariantContactTrustMode} YoEnum (live mode switch).
    * @param smoothingAlpha     exponential smoothing factor in [0, 1): p ← α·p_prev + (1−α)·p_raw.
    * @param loadEnterThreshold fraction of robot weight required to become trusted (Schmitt upper edge).
    * @param loadStayThreshold  fraction of robot weight below which trust is dropped (Schmitt lower edge).
    * @param trustDwellSeconds  time the enter/exit condition must hold before trust flips (both edges).
    */
   public FootSwitchContactProbabilityProvider(SideDependentList<? extends FootSwitchInterface> footSwitches,
                                               double estimatorDT,
                                               YoRegistry registry,
                                               double smoothingAlpha,
                                               double loadEnterThreshold,
                                               double loadStayThreshold,
                                               double trustDwellSeconds)
   {
      if (loadStayThreshold > loadEnterThreshold)
         throw new IllegalArgumentException("Schmitt band inverted: stay " + loadStayThreshold + " > enter " + loadEnterThreshold);
      this.footSwitches = footSwitches;
      this.smoothingAlpha = smoothingAlpha;
      this.loadEnterThreshold = loadEnterThreshold;
      this.loadStayThreshold = loadStayThreshold;
      this.trustDwellTicks = Math.max(1, (int) Math.round(trustDwellSeconds / estimatorDT));
      // Defaults to SCHMITT (ordinal 0), the production default. LEGACY/NONE remain reachable as A/B arms
      // by writing this YoEnum live, without a rebuild.
      this.trustMode = new YoEnum<>("invariantContactTrustMode", registry, TrustMode.class);
   }

   @Override
   public void update()
   {
      TrustMode mode = trustMode.getEnumValue();
      for (RobotSide side : RobotSide.values)
      {
         int index = side.ordinal();
         FootSwitchInterface footSwitch = footSwitches.get(side);
         footSwitch.update();

         boolean switchFiltered = footSwitch.hasFootHitGroundFiltered();
         double load = footSwitch.getFootLoadPercentage();

         // Schmitt trigger with dwell: the state-flipping condition must hold trustDwellTicks
         // consecutive ticks. NaN load (switch cannot compute it) degrades to switch-only trust:
         // !(NaN < threshold) passes both load tests. Advanced every tick in ALL modes so a live
         // mode switch never resumes from stale debounce state.
         boolean flipCondition = trusted[index] ? !(switchFiltered && !(load < loadStayThreshold))
                                                : switchFiltered && !(load < loadEnterThreshold);
         if (flipCondition)
         {
            if (++dwellCounter[index] >= trustDwellTicks)
            {
               trusted[index] = !trusted[index];
               dwellCounter[index] = 0;
            }
         }
         else
         {
            dwellCounter[index] = 0;
         }

         if (mode == TrustMode.NONE)
         {
            // Control arm: detection off, foot pinned fully trusted with no EMA lag.
            probability[index] = 1.0;
            continue;
         }

         // SCHMITT trusts the debounced state; LEGACY (pre-2026-07-17) trusts the raw filtered boolean.
         boolean fullTrust = mode == TrustMode.SCHMITT ? trusted[index] : switchFiltered;
         double pRaw;
         if (fullTrust)
            pRaw = 1.0;
         else if (footSwitch.hasFootHitGroundSensitive())
            pRaw = SENSITIVE_ONLY_PROBABILITY;
         else
            pRaw = 0.0;

         double smoothed = smoothingAlpha * probability[index] + (1.0 - smoothingAlpha) * pRaw;
         probability[index] = clamp(smoothed);
      }
   }

   @Override
   public double getContactProbability(RobotSide side)
   {
      return probability[side.ordinal()];
   }

   /** Exposed for tests and diagnostics: the debounced Schmitt-trigger trust state for {@code side}. */
   public boolean isTrusted(RobotSide side)
   {
      return trusted[side.ordinal()];
   }

   /** Sets the live trust mode (equivalent to writing the {@code invariantContactTrustMode} YoEnum). */
   public void setTrustMode(TrustMode mode)
   {
      trustMode.set(mode);
   }

   public TrustMode getTrustMode()
   {
      return trustMode.getEnumValue();
   }

   private static double clamp(double value)
   {
      return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
   }
}

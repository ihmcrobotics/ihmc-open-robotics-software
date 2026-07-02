package us.ihmc.stateEstimation.invariant_estimator;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;

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
 * <p>Probability mapping: {@code hasFootHitGroundFiltered() → 1}, else
 * {@code hasFootHitGroundSensitive() → 0.5} (fast but unconfirmed touchdown), else {@code 0}; then a
 * light exponential smoother so the estimator's soft-inflation laws (which exponentiate {@code 1 − p})
 * see a continuous signal instead of a step. The foot switches publish their own detector YoVariables
 * into the registry they were built with, so contact decisions remain fully inspectable.</p>
 *
 * <p>Requires the model the switches were built on to have current joint torques and frames when
 * {@link #update()} runs; the estimator guarantees this by calling the provider after its frame
 * update, with joints set upstream (by {@code InvariantMainStateEstimator#updateJoints()} when main,
 * by the DRC estimator's joint updater when secondary).</p>
 */
public class FootSwitchContactProbabilityProvider implements ContactProbabilityProvider
{
   /** Probability when only the sensitive (unconfirmed) detector reports contact. */
   private static final double SENSITIVE_ONLY_PROBABILITY = 0.5;
   /** Default exponential smoothing factor; at 1 kHz this is a ~5 ms time constant. */
   private static final double DEFAULT_SMOOTHING_ALPHA = 0.8;

   private final SideDependentList<? extends FootSwitchInterface> footSwitches;
   private final double smoothingAlpha;

   private final SideDependentList<Double> probability = new SideDependentList<>(1.0, 1.0);

   /** Builds the provider with default smoothing; probabilities start at 1 (feet planted at init). */
   public FootSwitchContactProbabilityProvider(SideDependentList<? extends FootSwitchInterface> footSwitches)
   {
      this(footSwitches, DEFAULT_SMOOTHING_ALPHA);
   }

   /**
    * @param footSwitches   the per-foot switches to poll; this provider calls their {@code update()}.
    * @param smoothingAlpha exponential smoothing factor in [0, 1): p ← α·p_prev + (1−α)·p_raw.
    */
   public FootSwitchContactProbabilityProvider(SideDependentList<? extends FootSwitchInterface> footSwitches, double smoothingAlpha)
   {
      this.footSwitches = footSwitches;
      this.smoothingAlpha = smoothingAlpha;
   }

   @Override
   public void update()
   {
      for (RobotSide side : RobotSide.values)
      {
         FootSwitchInterface footSwitch = footSwitches.get(side);
         footSwitch.update();

         double pRaw;
         if (footSwitch.hasFootHitGroundFiltered())
            pRaw = 1.0;
         else if (footSwitch.hasFootHitGroundSensitive())
            pRaw = SENSITIVE_ONLY_PROBABILITY;
         else
            pRaw = 0.0;

         double smoothed = smoothingAlpha * probability.get(side) + (1.0 - smoothingAlpha) * pRaw;
         probability.put(side, clamp(smoothed));
      }
   }

   @Override
   public double getContactProbability(RobotSide side)
   {
      return probability.get(side);
   }

   private static double clamp(double value)
   {
      return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
   }
}

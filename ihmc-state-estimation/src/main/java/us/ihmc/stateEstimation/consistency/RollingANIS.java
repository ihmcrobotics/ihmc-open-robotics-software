package us.ihmc.stateEstimation.consistency;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Onboard filter-consistency monitor for one measurement channel: a rolling average NIS per degree of freedom
 * (ANIS/d), published as a YoVariable so the per-channel NIS the evaluation protocol reports can be read straight
 * from a robot log, with no motion capture. A consistent channel reads 1; above 1 its innovation covariance S is too
 * small (the filter is overconfident on it), below 1 too large.
 * <p>
 * Each tick the channel's updates add their NIS and dimension ({@link #add}); {@link #commit} then pools them as
 * ΣNIS / Σd (so two feet, or nine joints, form one sample per tick) and folds that sample into an exponentially
 * weighted mean with weight {@code α = 1 − exp(−dt / T)}, T the window. Only ticks that added a finite NIS count:
 * a tick on which the channel did not run, or produced NaN, leaves the average untouched, so the window is T seconds
 * of APPLIED updates, not wall time. The mean is bias-corrected (weighted sum over accumulated weight), so it is NaN
 * until the first sample and exact from the first sample on, instead of rising from zero over the first window.
 * <p>
 * Logging only: it reads NIS values the filters already compute and feeds nothing back. Allocation-free.
 */
public class RollingANIS
{
   private final YoDouble anis;
   private final DoubleProvider windowSeconds;

   private double weightedSum = 0.0;
   private double accumulatedWeight = 0.0;
   private double tickNIS = 0.0;
   private double tickDof = 0.0;

   /**
    * @param name          the published ANIS/d variable's name.
    * @param windowSeconds the averaging time constant T (s), read at every commit; zero, negative or NaN disables
    *                      averaging (the variable then shows the latest tick's NIS/d).
    */
   public RollingANIS(String name, DoubleProvider windowSeconds, YoRegistry registry)
   {
      this.windowSeconds = windowSeconds;
      anis = new YoDouble(name, registry);
      anis.setToNaN();
   }

   /** Adds one update's NIS and its dimension to this tick's pooled sample. Non-finite NIS or {@code dof <= 0} is ignored. */
   public void add(double nis, double dof)
   {
      if (!Double.isFinite(nis) || !(dof > 0.0))
         return;
      tickNIS += nis;
      tickDof += dof;
   }

   /** Folds this tick's pooled sample, if any update was added, into the rolling mean. {@code dt} is the tick period (s). */
   public void commit(double dt)
   {
      if (tickDof > 0.0)
      {
         double sample = tickNIS / tickDof;
         double window = windowSeconds.getValue();
         double alpha = window > 0.0 ? 1.0 - Math.exp(-dt / window) : 1.0;
         weightedSum = (1.0 - alpha) * weightedSum + alpha * sample;
         accumulatedWeight = (1.0 - alpha) * accumulatedWeight + alpha;
         if (accumulatedWeight > 0.0)
            anis.set(weightedSum / accumulatedWeight);
      }
      tickNIS = 0.0;
      tickDof = 0.0;
   }

   /** One update per tick: {@link #add} then {@link #commit}. */
   public void update(double nis, double dof, double dt)
   {
      add(nis, dof);
      commit(dt);
   }

   /** Forgets the history; the published value goes back to NaN. */
   public void reset()
   {
      weightedSum = 0.0;
      accumulatedWeight = 0.0;
      tickNIS = 0.0;
      tickDof = 0.0;
      anis.setToNaN();
   }

   public double getValue()
   {
      return anis.getDoubleValue();
   }

   public YoDouble getYoVariable()
   {
      return anis;
   }
}

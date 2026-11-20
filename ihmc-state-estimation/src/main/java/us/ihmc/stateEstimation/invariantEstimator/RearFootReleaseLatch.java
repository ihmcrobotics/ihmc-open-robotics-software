package us.ihmc.stateEstimation.invariantEstimator;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Decides when to stop trusting the REAR foot's kinematic velocity in double support (see
 * {@code invariantKinematicVelocityRearRelease} in {@link InvariantEKFStateEstimator}).
 * <p>
 * A foot is released when it is trusted, the other foot is trusted, its load is below the release load, below the
 * other foot's load, and has dropped by at least {@code minDrop} over the last {@link #RATE_WINDOW} ticks (it is
 * unloading at walking speed: a weight transfer loses about 0.8 of the load in 100-200 ms, i.e. 0.08-0.16 per 20 ms,
 * while standing sway moves it by a few thousandths; without the drop requirement the lighter foot stood released
 * 76-86% of the standing time in replay). The release latches until the
 * foot lifts (load below the untrust load or non-finite), the other foot stops being trusted, or the feature is turned
 * off -- the load-gate hysteresis would otherwise re-trust it at once, since its load is still above the trust load.
 * The loading front foot is never released (its load is rising), and the lighter-foot condition means the two feet
 * are never released together. Allocation-free after construction.
 * </p>
 */
public class RearFootReleaseLatch
{
   public static final int RATE_WINDOW = 20; // ticks

   private final SideDependentList<double[]> history = new SideDependentList<>(new double[RATE_WINDOW], new double[RATE_WINDOW]);
   private final SideDependentList<boolean[]> released = new SideDependentList<>(new boolean[1], new boolean[1]);
   private int count = 0;

   /**
    * @param minDrop                   the load drop over {@link #RATE_WINDOW} ticks that counts as unloading (≥ 0).
    * @param leftLoad, rightLoad       this tick's foot load fractions (NaN allowed).
    * @param leftTrusted, rightTrusted each foot's load-gate trust from the previous tick.
    */
   public void update(boolean enabled,
                      double releaseLoad,
                      double untrustLoad,
                      double minDrop,
                      double leftLoad,
                      double rightLoad,
                      boolean leftTrusted,
                      boolean rightTrusted)
   {
      int slot = count % RATE_WINDOW;
      boolean full = count >= RATE_WINDOW;
      for (RobotSide side : RobotSide.values)
      {
         double load = side == RobotSide.LEFT ? leftLoad : rightLoad;
         double otherLoad = side == RobotSide.LEFT ? rightLoad : leftLoad;
         boolean trustedThis = side == RobotSide.LEFT ? leftTrusted : rightTrusted;
         boolean trustedOther = side == RobotSide.LEFT ? rightTrusted : leftTrusted;
         double[] h = history.get(side);
         double loadWindowAgo = h[slot];
         boolean[] r = released.get(side);
         if (!enabled || !Double.isFinite(load) || load < untrustLoad || !trustedOther)
            r[0] = false;
         else if (!r[0] && full && trustedThis && load < releaseLoad && loadWindowAgo - load >= minDrop && load < otherLoad)
            r[0] = true;
         h[slot] = Double.isFinite(load) ? load : Double.NaN;
      }
      count++;
   }

   public boolean isReleased(RobotSide side)
   {
      return released.get(side)[0];
   }
}

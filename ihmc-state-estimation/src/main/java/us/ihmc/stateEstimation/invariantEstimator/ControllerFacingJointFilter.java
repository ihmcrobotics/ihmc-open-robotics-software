package us.ihmc.stateEstimation.invariantEstimator;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.HashMap;
import java.util.Map;

/**
 * The anti-damping filter on the joint states the invariant main estimator hands to the controller (2026-07-16).
 * <p>
 * With the measured ~19 ms loop delay, the whole-body controller's velocity feedback on the pitch-plane leg joints
 * (hips and knees, the two measured energy injectors) injects energy around 12-14 Hz (+2.7 W measured, left knee, log
 * jointKF_Osc_0716). The old ALPHA_COMPLEMENTARY chain attenuated that band implicitly; the joint-level KF passes it at
 * unity gain. This filter removes it from what the controller reads.
 * </p>
 * <p>
 * It is for the controller only. The estimator itself must keep the full-bandwidth signal: the invariant filter's
 * kinematic velocity built from band-stopped hip/knee velocities lags by ~12 ms after touchdown and biases the vertical
 * velocity (MuJoCo perceptive walking, 2026-11-16: kinematic-velocity error rms 16 vs 1.3 mm/s, height drift -8.7 vs
 * -3.3 mm/s). {@link #isInvariantFilterUsingFilteredStates()} only exists to reproduce results from before that fix.
 * </p>
 * <p>
 * Gating, all live YoVariables:
 * <ul>
 * <li>{@code controllerFacingQdNotchEnabled}: master switch (default on).</li>
 * <li>{@code controllerFacingQdFilterInRL}: whether the filter also applies while an RL controller state is active
 * (default off). The RL policies were trained, and run on DRC, without it, so they read the unfiltered states.</li>
 * <li>{@code controllerFacingRLStateActive}: set from the high-level controller state through
 * {@link #setRLStateActive(boolean)}. Until it is set, the state counts as non-RL, which is the filter's behaviour
 * before the gate existed.</li>
 * <li>{@code controllerFacingQdFilterActive}: read-only, whether the filter is applied this tick.</li>
 * <li>{@code invariantUsesControllerFacingStates}: give the invariant filter the controller-facing states too
 * (default off; the behaviour before 2026-11-16).</li>
 * </ul>
 * Every biquad runs every tick whatever the gates say, so its state stays warm and switching is bumpless.
 * </p>
 */
public class ControllerFacingJointFilter
{
   public enum FilterType
   {
      NONE, NOTCH, LOW_PASS, BAND_STOP
   }

   /** Which joint quantities the selected filter applies to. */
   public enum FilteredQuantity
   {
      JOINT_VELOCITIES, JOINT_POSITIONS, BOTH
   }

   // NOTCH (11.8 Hz, Q=2): surgical, <5 deg phase at 3 Hz; hardware showed the mode slides to the notch shoulder
   // (13.8 Hz): a 19 ms loop delay anti-damps a band, not a line. LOW_PASS (2nd-order Butterworth, 7 Hz): rolls off
   // the whole band at ~15-20 deg phase cost at 3 Hz; it broke balance on hardware. BAND_STOP (default): a wide notch,
   // f0 = 13.5 Hz, Q = 1.2, -3 dB stopband 9.0-20.25 Hz, ~-11 deg at 3 Hz.
   public static final double NOTCH_CENTER_HZ = 11.8;
   public static final double NOTCH_Q = 2.0;
   public static final double LOW_PASS_CORNER_HZ = 7.0;
   public static final double BAND_STOP_CENTER_HZ = 13.5;
   public static final double BAND_STOP_Q = 1.2;

   private final YoBoolean enabled;
   private final YoBoolean filterInRL;
   private final YoBoolean rlStateActive;
   private final YoBoolean filterActive;
   private final YoBoolean invariantUsesFilteredStates;
   private final YoEnum<FilterType> filterType;
   private final YoEnum<FilteredQuantity> filteredQuantity;

   private final Map<String, JointFilters> jointFilters = new HashMap<>();

   private double filteredPosition;
   private double filteredVelocity;

   /** @param jointNames the joints to filter; any other joint passes through */
   public ControllerFacingJointFilter(Iterable<String> jointNames, double dt, YoRegistry registry)
   {
      enabled = new YoBoolean("controllerFacingQdNotchEnabled", registry);
      enabled.set(true);
      filterInRL = new YoBoolean("controllerFacingQdFilterInRL", registry);
      filterInRL.set(false);
      rlStateActive = new YoBoolean("controllerFacingRLStateActive", registry);
      filterActive = new YoBoolean("controllerFacingQdFilterActive", registry);
      invariantUsesFilteredStates = new YoBoolean("invariantUsesControllerFacingStates", registry);
      invariantUsesFilteredStates.set(false);
      filterType = new YoEnum<>("controllerFacingQdFilterType", registry, FilterType.class);
      filterType.set(FilterType.BAND_STOP);
      filteredQuantity = new YoEnum<>("controllerFacingFilteredQuantity", registry, FilteredQuantity.class);
      filteredQuantity.set(FilteredQuantity.JOINT_VELOCITIES);

      for (String jointName : jointNames)
         jointFilters.put(jointName, new JointFilters(dt));
   }

   /** The joints the filter is built for on a legged robot: every hip and knee pitch joint. */
   public static boolean isHipOrKneePitch(String jointName)
   {
      return jointName.contains("HIP_Y") || jointName.contains("KNEE_Y");
   }

   /** Called with the high-level controller state, e.g. from the estimator thread's controller-state listener. */
   public void setRLStateActive(boolean active)
   {
      rlStateActive.set(active);
   }

   /** Evaluates the gates once per tick, before the joints are filtered. */
   public void updateGate()
   {
      filterActive.set(enabled.getBooleanValue() && (!rlStateActive.getBooleanValue() || filterInRL.getBooleanValue()));
   }

   public boolean isActive()
   {
      return filterActive.getBooleanValue();
   }

   public boolean isInvariantFilterUsingFilteredStates()
   {
      return invariantUsesFilteredStates.getBooleanValue();
   }

   public boolean filtersJoint(String jointName)
   {
      return jointFilters.containsKey(jointName);
   }

   /**
    * Advances the joint's filters with the estimator's position and velocity and packs the controller-facing values,
    * read with {@link #getFilteredPosition()} and {@link #getFilteredVelocity()}. A joint the filter is not built for,
    * or any joint while the filter is inactive, gets its input back.
    */
   public void update(String jointName, double position, double velocity)
   {
      filteredPosition = position;
      filteredVelocity = velocity;
      JointFilters filters = jointFilters.get(jointName);
      if (filters == null)
         return;

      filters.update(position, velocity);
      if (!filterActive.getBooleanValue())
         return;

      FilteredQuantity quantity = filteredQuantity.getEnumValue();
      boolean applyVelocity = quantity != FilteredQuantity.JOINT_POSITIONS;
      boolean applyPosition = quantity != FilteredQuantity.JOINT_VELOCITIES;
      switch (filterType.getEnumValue())
      {
         case NOTCH ->
         {
            if (applyVelocity)
               filteredVelocity = filters.notchedQd;
            if (applyPosition)
               filteredPosition = filters.notchedQ;
         }
         case LOW_PASS ->
         {
            if (applyVelocity)
               filteredVelocity = filters.lowPassedQd;
            if (applyPosition)
               filteredPosition = filters.lowPassedQ;
         }
         case BAND_STOP ->
         {
            if (applyVelocity)
               filteredVelocity = filters.bandStoppedQd;
            if (applyPosition)
               filteredPosition = filters.bandStoppedQ;
         }
         case NONE ->
         {
         }
      }
   }

   public double getFilteredPosition()
   {
      return filteredPosition;
   }

   public double getFilteredVelocity()
   {
      return filteredVelocity;
   }

   private static final class JointFilters
   {
      private final Biquad notchQd, lowPassQd, bandStopQd, notchQ, lowPassQ, bandStopQ;
      private double notchedQd, lowPassedQd, bandStoppedQd, notchedQ, lowPassedQ, bandStoppedQ;

      JointFilters(double dt)
      {
         notchQd = Biquad.notch(NOTCH_CENTER_HZ, NOTCH_Q, dt);
         lowPassQd = Biquad.lowPass(LOW_PASS_CORNER_HZ, dt);
         bandStopQd = Biquad.notch(BAND_STOP_CENTER_HZ, BAND_STOP_Q, dt);
         notchQ = Biquad.notch(NOTCH_CENTER_HZ, NOTCH_Q, dt);
         lowPassQ = Biquad.lowPass(LOW_PASS_CORNER_HZ, dt);
         bandStopQ = Biquad.notch(BAND_STOP_CENTER_HZ, BAND_STOP_Q, dt);
      }

      void update(double position, double velocity)
      {
         notchedQd = notchQd.update(velocity);
         lowPassedQd = lowPassQd.update(velocity);
         bandStoppedQd = bandStopQd.update(velocity);
         notchedQ = notchQ.update(position);
         lowPassedQ = lowPassQ.update(position);
         bandStoppedQ = bandStopQ.update(position);
      }
   }

   /** RBJ biquad, direct form 1; fixed coefficients (shape, f0, Q, dt known at construction). */
   static final class Biquad
   {
      private final double b0, b1, b2, a1, a2;
      private double x1, x2, y1, y2;

      private Biquad(double b0, double b1, double b2, double a1, double a2)
      {
         this.b0 = b0;
         this.b1 = b1;
         this.b2 = b2;
         this.a1 = a1;
         this.a2 = a2;
      }

      static Biquad notch(double centerHz, double q, double dt)
      {
         double w0 = 2.0 * Math.PI * centerHz * dt;
         double alpha = Math.sin(w0) / (2.0 * q);
         double a0 = 1.0 + alpha;
         return new Biquad(1.0 / a0, -2.0 * Math.cos(w0) / a0, 1.0 / a0, -2.0 * Math.cos(w0) / a0, (1.0 - alpha) / a0);
      }

      static Biquad lowPass(double cornerHz, double dt)
      {
         double w0 = 2.0 * Math.PI * cornerHz * dt;
         double alpha = Math.sin(w0) / (2.0 * Math.sqrt(0.5)); // Q = 1/sqrt(2): Butterworth
         double a0 = 1.0 + alpha;
         double c = 1.0 - Math.cos(w0);
         return new Biquad(0.5 * c / a0, c / a0, 0.5 * c / a0, -2.0 * Math.cos(w0) / a0, (1.0 - alpha) / a0);
      }

      double update(double x)
      {
         double y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
         x2 = x1;
         x1 = x;
         y2 = y1;
         y1 = y;
         return y;
      }
   }
}

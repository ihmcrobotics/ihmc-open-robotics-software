package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.DoubleProvider;

/**
 * [TMECH] Time alignment of the inputs of the inertial estimator's measurement equation
 * {@code tau = tau_nom(q, qd, qdd) + Y(q, qd, qdd) pi - sum J^T w}.
 * <p>
 * The estimator runs at the controller rate and has no measured joint accelerations, so {@code qdd} is a filtered backward
 * difference of the sampled velocities: {@code LPF[(qd_k - qd_{k-1}) / dt]}. That signal describes the robot half a tick
 * plus the low-pass group delay in the PAST (about 9 ms at 100 Hz with the 20 Hz filter), while {@code q}, {@code qd}, the
 * measured torques and the foot wrenches are the current samples. The regressor therefore pairs today's torque with
 * yesterday's acceleration. In motion that mismatch is a systematic, motion-correlated residual (it grows with the jerk of
 * the motion), which the filter absorbs as parameter wander.
 * </p>
 * <p>
 * This class removes the mismatch by passing EVERY input through the same linear filter chain as the acceleration, so all of
 * them describe the same past instant:
 * <ul>
 * <li>{@link Mode#MIDPOINT}: {@code x = LPF[(x_k + x_{k-1}) / 2]} and {@code qdd = LPF[(qd_k - qd_{k-1}) / dt]}; the two-tap
 * average has exactly the half-tick delay of the backward difference. Latency dt/2 + LPF.</li>
 * <li>{@link Mode#CENTRAL}: {@code x = LPF[x_{k-1}]} and {@code qdd = LPF[(qd_k - qd_{k-2}) / (2 dt)]}, a central difference
 * (half the acceleration noise of the backward difference). Latency dt + LPF.</li>
 * </ul>
 * Because both sides of the (in pi linear) measurement equation see the same filter, the low-pass becomes a pure noise knob
 * that no longer biases the estimate. The floating-base pose is not low-passed (the quaternion is not a vector space); in
 * CENTRAL it is delayed by the same one tick, in MIDPOINT it is the current sample. The base turns slowly, so the residual
 * pose mismatch is the LPF delay times the base rate.
 * </p>
 * Default off ({@code -Dalex.inertiaEstimator.alignMode=OFF}).
 */
public class InertialMeasurementAligner
{
   public enum Mode
   {
      OFF, MIDPOINT, CENTRAL,
      /**
       * As MIDPOINT for q and qd, but the torques and foot wrenches are the CURRENT samples: when the torque is a zero-order
       * hold of the controller output updated once per tick (as the MPC torque is), the sample at t_k already IS the mean
       * over the backward-difference window [t_{k-1}, t_k].
       */
      MIDHOLD,
      /** As CENTRAL for q and qd, but the torques and foot wrenches are the mean of the samples at t_k and t_{k-1}: for a
       * per-tick zero-order hold, the mean over the central-difference window [t_{k-2}, t_k]. */
      CENHOLD
   }

   private final Mode mode;
   private final int rootConfigurationSize;
   private final DoubleProvider alpha;
   private final DoubleProvider dt;

   private final DMatrixRMaj qPrev, vPrev, vPrev2, tauPrev;
   private final SideDependentList<DMatrixRMaj> wPrev = new SideDependentList<>();

   private final DMatrixRMaj qRaw, vRaw, aRaw, tauRaw;
   private final SideDependentList<DMatrixRMaj> wRaw = new SideDependentList<>();

   private final DMatrixRMaj qOut, vOut, aOut, tauOut;
   private final SideDependentList<DMatrixRMaj> wOut = new SideDependentList<>();

   private int samples = 0;

   /**
    * @param nq                    configuration-space size (root pose + one-DoF joints)
    * @param nv                    velocity-space size
    * @param rootConfigurationSize number of leading configuration rows that belong to the floating base (7), not low-passed
    * @param alpha                 the low-pass alpha shared by all signals (the acceleration filter's alpha)
    */
   public InertialMeasurementAligner(Mode mode, int nq, int nv, int rootConfigurationSize, DoubleProvider alpha, DoubleProvider dt)
   {
      this.mode = mode;
      this.rootConfigurationSize = rootConfigurationSize;
      this.alpha = alpha;
      this.dt = dt;
      qPrev = new DMatrixRMaj(nq, 1);
      qRaw = new DMatrixRMaj(nq, 1);
      qOut = new DMatrixRMaj(nq, 1);
      vPrev = new DMatrixRMaj(nv, 1);
      vPrev2 = new DMatrixRMaj(nv, 1);
      tauPrev = new DMatrixRMaj(nv, 1);
      vRaw = new DMatrixRMaj(nv, 1);
      aRaw = new DMatrixRMaj(nv, 1);
      tauRaw = new DMatrixRMaj(nv, 1);
      vOut = new DMatrixRMaj(nv, 1);
      aOut = new DMatrixRMaj(nv, 1);
      tauOut = new DMatrixRMaj(nv, 1);
      for (RobotSide side : RobotSide.values)
      {
         wPrev.put(side, new DMatrixRMaj(6, 1));
         wRaw.put(side, new DMatrixRMaj(6, 1));
         wOut.put(side, new DMatrixRMaj(6, 1));
      }
   }

   public static Mode parseMode(String value)
   {
      if (value == null || value.isBlank())
         return Mode.OFF;
      return Mode.valueOf(value.trim().toUpperCase());
   }

   public Mode getMode()
   {
      return mode;
   }

   /** Feed this tick's raw samples (all current); afterwards read the aligned, filtered signals with the getters. */
   public void update(DMatrixRMaj q, DMatrixRMaj v, DMatrixRMaj tau, SideDependentList<DMatrixRMaj> w)
   {
      if (samples == 0)
      {
         qPrev.set(q);
         vPrev.set(v);
         vPrev2.set(v);
         tauPrev.set(tau);
         for (RobotSide side : RobotSide.values)
            wPrev.get(side).set(w.get(side));
      }

      double h = dt.getValue();
      boolean central = mode == Mode.CENTRAL || mode == Mode.CENHOLD;
      if (central)
      {
         qRaw.set(qPrev);
         vRaw.set(vPrev);
         for (int i = 0; i < v.getNumRows(); i++)
            aRaw.set(i, 0, samples < 2 ? 0.0 : (v.get(i, 0) - vPrev2.get(i, 0)) / (2.0 * h));
      }
      else // MIDPOINT, MIDHOLD
      {
         for (int i = 0; i < q.getNumRows(); i++)
            qRaw.set(i, 0, i < rootConfigurationSize ? q.get(i, 0) : 0.5 * (q.get(i, 0) + qPrev.get(i, 0)));
         for (int i = 0; i < v.getNumRows(); i++)
         {
            vRaw.set(i, 0, 0.5 * (v.get(i, 0) + vPrev.get(i, 0)));
            aRaw.set(i, 0, samples < 1 ? 0.0 : (v.get(i, 0) - vPrev.get(i, 0)) / h);
         }
      }
      // torques and foot wrenches: current sample, previous sample, or their mean (see the Mode docs)
      double cNow = switch (mode)
      {
         case CENTRAL -> 0.0;
         case MIDHOLD -> 1.0;
         default -> 0.5;  // MIDPOINT, CENHOLD
      };
      for (int i = 0; i < v.getNumRows(); i++)
         tauRaw.set(i, 0, cNow * tau.get(i, 0) + (1.0 - cNow) * tauPrev.get(i, 0));
      for (RobotSide side : RobotSide.values)
         for (int i = 0; i < 6; i++)
            wRaw.get(side).set(i, 0, cNow * w.get(side).get(i, 0) + (1.0 - cNow) * wPrev.get(side).get(i, 0));

      // The same first-order low-pass on every signal (root pose rows excepted).
      double a = samples == 0 ? 0.0 : alpha.getValue();
      for (int i = 0; i < qRaw.getNumRows(); i++)
         qOut.set(i, 0, i < rootConfigurationSize ? qRaw.get(i, 0) : a * qOut.get(i, 0) + (1.0 - a) * qRaw.get(i, 0));
      lowPass(a, vRaw, vOut);
      lowPass(a, tauRaw, tauOut);
      // The acceleration filter is seeded with the first valid difference.
      int firstDifference = central ? 2 : 1;
      lowPass(samples <= firstDifference ? 0.0 : a, aRaw, aOut);
      for (RobotSide side : RobotSide.values)
         lowPass(a, wRaw.get(side), wOut.get(side));

      // history
      vPrev2.set(vPrev);
      vPrev.set(v);
      qPrev.set(q);
      tauPrev.set(tau);
      for (RobotSide side : RobotSide.values)
         wPrev.get(side).set(w.get(side));
      samples++;
   }

   private static void lowPass(double a, DMatrixRMaj raw, DMatrixRMaj out)
   {
      for (int i = 0; i < raw.getNumRows(); i++)
         out.set(i, 0, a * out.get(i, 0) + (1.0 - a) * raw.get(i, 0));
   }

   public DMatrixRMaj getConfiguration()
   {
      return qOut;
   }

   public DMatrixRMaj getVelocity()
   {
      return vOut;
   }

   public DMatrixRMaj getAcceleration()
   {
      return aOut;
   }

   public DMatrixRMaj getEffort()
   {
      return tauOut;
   }

   public DMatrixRMaj getWrench(RobotSide side)
   {
      return wOut.get(side);
   }
}

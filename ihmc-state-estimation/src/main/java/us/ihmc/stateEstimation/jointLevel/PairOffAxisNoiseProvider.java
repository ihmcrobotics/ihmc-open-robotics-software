package us.ihmc.stateEstimation.jointLevel;

import java.util.Arrays;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;

import us.ihmc.log.LogTools;

/**
 * Per-tick, per-pair measurement noise driven by the IMU array's own off-axis residual.
 *
 * <h2>What it measures</h2>
 * An IMU pair spanning a joint measures {@code z_e = omega_child - R_rel omega_parent}. If the link
 * between the two IMUs is rigid, that vector MUST lie in the column space of the pair's relative
 * angular Jacobian {@code J_e(q)} -- a line for a 1-DoF pair, a plane for a 2-DoF one. Whatever is
 * orthogonal to that span carries no joint information at all; it is link flex, mount shift,
 * vibration and bias. So
 *
 * <pre>    r_off_e(t) = (I - P_e(q)) z_e(t),   P_e = projector onto colspace(J_e)</pre>
 *
 * is a direct, online, ground-truth-free reading of how badly the rigid-body model this filter is
 * built on is being violated right now, on that pair. The filter's noise model is supposed to say
 * how much to trust the measurement; this says it from the measurement itself rather than from a
 * constant chosen once at a desk.
 *
 * <p>The residual is taken from the SAME {@code z_e} and {@code J_e} the stacked measurement is
 * assembled from, before any update -- so nothing here can be contaminated by the estimate it is
 * about to reweight.</p>
 *
 * <h2>Slow versus fast</h2>
 * Not all of {@code r_off} is the violation we want. A gyro bias and a constant mount misalignment
 * both land in it and are SLOW; impact flex and vibration are FAST. Only the fast part should drive
 * the noise: a constant misalignment does not make this tick's on-axis measurement any worse than
 * the last one's, and folding it in would add a constant offset to every tick, which is just the
 * hand-tuned constant again wearing a costume. The split is a causal trailing MEDIAN, subtracted:
 *
 * <pre>    slow_e(t) = median(r_off_e(t-W+1 .. t)) per component,   fast_e(t) = r_off_e(t) - slow_e(t)</pre>
 *
 * Median and not a mean because the fast part is impulsive -- heel strike -- and an impulse drags a
 * moving mean, and therefore the "slow" estimate, toward itself. That is exactly the leakage the
 * split exists to prevent.
 *
 * <h2>The inflation, and why it is additive</h2>
 * <pre>    extra_e(t) = (|fast_e(t)| / s_e)^2 * sigma0_e</pre>
 * One when the robot is still, so a standing window is untouched, and quadratic because a
 * gyro-difference error enters the innovation linearly and the NIS quadratically. It is ADDED to
 * pair {@code e}'s own 3x3 diagonal block of {@code Rg} rather than multiplying it: {@code Rg = L
 * Sigma L^T} is correlated across pairs that share an IMU, and scaling one diagonal block of a
 * correlated positive-definite matrix can destroy positive-definiteness, while adding a
 * non-negative diagonal cannot.
 *
 * <h2>Strictly causal by construction</h2>
 * The obvious objection to any measurement-driven R is circularity: if {@code R(t)} is built from
 * tick {@code t}'s own measurement, it is correlated with the innovation it weights and
 * systematically down-weights exactly the ticks whose innovations are large. This class answers it
 * structurally rather than by argument -- what it emits at tick {@code t} is the trailing MAX over
 * the previous {@code causalWindow} ticks, STRICTLY EXCLUDING {@code t}. The current tick's residual
 * is recorded and only becomes visible to the filter from the next tick on. Max and not mean because
 * the disturbance is impulsive and a mean over a window long enough to exclude the present would
 * smear a heel strike into nothing; the max holds the recent worst case, which is both the
 * conservative reading and what an online implementation would do.
 *
 * <p>The offline study found the causal variant BEATS the same-tick one on held-out windows, so
 * nothing is given up by taking it -- which is why it is the only form offered here.</p>
 *
 * <h2>Fitted constants</h2>
 * {@code s_e = alpha * median_fit(|fast_e|)} -- one global {@code alpha} plus one number per pair,
 * all from a fit window that is never a scored window. They arrive here already multiplied out.
 * {@code sigma0_e} is a static per-pair isotropic baseline, {@code (tr(Sigma_child) +
 * tr(Sigma_parent)) / 3}; it is rotation-invariant, hence constant. It is supplied rather than
 * recomputed so that this arm reproduces the offline one exactly, but {@link #checkSigmaZero} lets
 * the caller compare it against the filter's own Sigma and say so if they disagree -- a silent
 * disagreement there would make a Java-produced arm differ from its Python twin for a reason that
 * has nothing to do with the variable under study.
 *
 * <h2>Not yet real-time</h2>
 * Real-time: the trailing median is a {@link SortedSlidingWindow} per pair and component (one binary
 * search and one array shift per sample, bit-identical to re-sorting the window), the SVD works in
 * preallocated storage, and {@link #update} allocates nothing after its first call per chain shape. It
 * previously re-sorted every window every tick and copied the Jacobian, which is fine for log replay and
 * not for a 1 kHz control loop.
 */
public class PairOffAxisNoiseProvider
{
   /** Singular values below this fraction of the largest are not part of the span. */
   private static final double RANK_TOL = 1.0e-8;

   private final int numberOfPairs;
   private final String[] pairChains;
   private final double[] scale;
   private final double[] sigmaZero;
   private final int medianWindow;
   private final int causalWindow;

   private final SingularValueDecomposition_F64<DMatrixRMaj>[] svd;
   private final DMatrixRMaj zPair = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj offAxis = new DMatrixRMaj(3, 1);

   /** Trailing history of the off-axis vector, per pair, per component: [pair][component][slot]. */
   private final SortedSlidingWindow[][] offHistory;
   private final DMatrixRMaj[] jacobianCopy;
   private final DMatrixRMaj[] uStorage;
   /** Trailing history of the raw same-tick inflation, per pair. Emission reads this, never the current tick. */
   private final double[][] extraHistory;
   private final double[] emitted;

   private long tick = 0L;
   private boolean warnedSigmaZeroMismatch = false;

   @SuppressWarnings("unchecked")
   public PairOffAxisNoiseProvider(String[] pairChains, double[] scale, double[] sigmaZero, int medianWindow, int causalWindow)
   {
      this.numberOfPairs = pairChains.length;
      if (scale.length != numberOfPairs || sigmaZero.length != numberOfPairs)
         throw new IllegalArgumentException("scale and sigma0 must have one entry per pair (" + numberOfPairs + "), got " + scale.length + " and "
               + sigmaZero.length);
      if (medianWindow < 1 || causalWindow < 1)
         throw new IllegalArgumentException("windows must be at least one tick, got median=" + medianWindow + " causal=" + causalWindow);
      for (int e = 0; e < numberOfPairs; e++)
      {
         // A zero or non-finite scale would divide the residual into an infinity and silently take the
         // filter's whole stacked channel out of service on the first moving tick.
         if (!(scale[e] > 0.0) || !Double.isFinite(scale[e]))
            throw new IllegalArgumentException("pair " + pairChains[e] + " scale must be finite and positive, was " + scale[e]);
         if (!(sigmaZero[e] > 0.0) || !Double.isFinite(sigmaZero[e]))
            throw new IllegalArgumentException("pair " + pairChains[e] + " sigma0 must be finite and positive, was " + sigmaZero[e]);
      }

      this.pairChains = pairChains.clone();
      this.scale = scale.clone();
      this.sigmaZero = sigmaZero.clone();
      this.medianWindow = medianWindow;
      this.causalWindow = causalWindow;

      this.svd = new SingularValueDecomposition_F64[numberOfPairs];
      this.offHistory = new SortedSlidingWindow[numberOfPairs][3];
      for (int e = 0; e < numberOfPairs; e++)
         for (int r = 0; r < 3; r++)
            offHistory[e][r] = new SortedSlidingWindow(medianWindow);
      this.jacobianCopy = new DMatrixRMaj[numberOfPairs];
      this.uStorage = new DMatrixRMaj[numberOfPairs];
      // causalWindow + 1 slots: the emitted value reads causalWindow PAST ticks while the current
      // tick already occupies its own slot, so a ring of exactly causalWindow would overwrite the
      // oldest of them before it had been read.
      this.extraHistory = new double[numberOfPairs][causalWindow + 1];
      this.emitted = new double[numberOfPairs];
   }

   /** The pair chains ("parentImu-&gt;childImu") the fitted constants were keyed to, in artifact order. */
   public String[] getPairChains()
   {
      return pairChains.clone();
    }

   /**
    * Compares the artifact's {@code sigma0_e} against the value implied by the filter's own gyro noise
    * covariance and warns once if they disagree. Not an exception: the artifact's number is the one
    * that reproduces the offline arm, and refusing to run would be worse than saying so out loud.
    *
    * @param filterSigmaZero per-pair {@code (tr(Sigma_child) + tr(Sigma_parent)) / 3} from this filter.
    */
   public void checkSigmaZero(double[] filterSigmaZero)
   {
      if (warnedSigmaZeroMismatch || filterSigmaZero.length != numberOfPairs)
         return;
      for (int e = 0; e < numberOfPairs; e++)
      {
         double relative = Math.abs(filterSigmaZero[e] - sigmaZero[e]) / Math.max(sigmaZero[e], 1.0e-30);
         if (relative > 1.0e-6)
         {
            warnedSigmaZeroMismatch = true;
            LogTools.warn("PairOffAxisNoiseProvider: pair " + pairChains[e] + " sigma0 from the artifact is " + sigmaZero[e]
                  + " but this filter's own gyro Sigma implies " + filterSigmaZero[e] + " (relative " + relative
                  + "). The artifact's value is being used, so this arm still reproduces the offline one -- but the two "
                  + "noise models have drifted apart and a comparison against the offline arm now spans that difference too. "
                  + "Reported once.");
            return;
         }
      }
   }

   /**
    * Records this tick's off-axis residual for every pair and returns the inflation to apply NOW,
    * which is built only from earlier ticks.
    *
    * @param zg          the stacked measurement; rows {@code [3e, 3e+3)} are pair {@code e}'s {@code z_e}.
    * @param jacobians   per-pair relative angular Jacobian, 3 x (path joints), in the same frame as {@code z_e}.
    * @return the per-pair additive variance, valid until the next call. Never null; all zeros on tick 0.
    */
   public double[] update(DMatrixRMaj zg, DMatrixRMaj[] jacobians)
   {
      if (jacobians.length != numberOfPairs)
         throw new IllegalArgumentException("expected " + numberOfPairs + " pair Jacobians, got " + jacobians.length);

      for (int e = 0; e < numberOfPairs; e++)
      {
         for (int r = 0; r < 3; r++)
            zPair.set(r, 0, zg.get(3 * e + r, 0));

         projectOffAxis(e, jacobians[e], zPair, offAxis);

         // Fast part: this tick's off-axis minus the trailing per-component median, which is the
         // slow bias/misalignment content the inflation must not respond to.
         double fastSquared = 0.0;
         for (int r = 0; r < 3; r++)
         {
            offHistory[e][r].add(offAxis.get(r, 0));
            double fast = offAxis.get(r, 0) - offHistory[e][r].median();
            fastSquared += fast * fast;
         }

         double ratio = Math.sqrt(fastSquared) / scale[e];
         double raw = ratio * ratio * sigmaZero[e];
         // A non-finite z (a dropped IMU frame) must not poison the history: it would make the
         // trailing max non-finite for the whole causal window and take the channel out of service
         // long after the bad tick is gone. Treat it as "no information", which is zero inflation.
         extraHistory[e][(int) (tick % extraHistory[e].length)] = Double.isFinite(raw) ? raw : 0.0;

         emitted[e] = trailingMaxExcludingNow(extraHistory[e], tick);
      }

      tick++;
      return emitted;
   }

   /** Clears the history so a second replay in the same process does not inherit the first one's tail. */
   public void reset()
   {
      tick = 0L;
      for (int e = 0; e < numberOfPairs; e++)
      {
         for (int r = 0; r < 3; r++)
            offHistory[e][r].clear();
         Arrays.fill(extraHistory[e], 0.0);
      }
      Arrays.fill(emitted, 0.0);
   }

   /** {@code off = (I - U_keep U_keep^T) z}, with {@code U_keep} an orthonormal basis of colspace(J). */
   private void projectOffAxis(int pair, DMatrixRMaj jacobian, DMatrixRMaj z, DMatrixRMaj offToPack)
   {
      offToPack.set(z);
      int columns = jacobian.getNumCols();
      if (columns == 0)
         return; // no joint in the span: everything the pair sees is off-axis, which is the correct reading.

      if (svd[pair] == null || svd[pair].numCols() != columns)
      {
         // Built once per chain shape; every later tick reuses the decomposition and both storages.
         svd[pair] = DecompositionFactory_DDRM.svd(3, columns, true, false, true);
         jacobianCopy[pair] = new DMatrixRMaj(3, columns);
         uStorage[pair] = new DMatrixRMaj(3, Math.min(3, columns));
      }
      jacobianCopy[pair].set(jacobian);
      if (!svd[pair].decompose(jacobianCopy[pair])) // decompose() is destructive, hence the copy
      {
         // No span means no on-axis part to remove; off = z is already packed. Silent rather than
         // thrown because one failed SVD must not stop a replay, and the reading is still defined.
         return;
      }

      DMatrixRMaj u = svd[pair].getU(uStorage[pair], false);
      double[] singular = svd[pair].getSingularValues();
      double largest = 0.0;
      for (double s : singular)
         largest = Math.max(largest, s);

      for (int k = 0; k < u.getNumCols() && k < singular.length; k++)
      {
         if (singular[k] <= RANK_TOL * Math.max(largest, 1.0e-30))
            continue;
         double component = 0.0;
         for (int r = 0; r < 3; r++)
            component += u.get(r, k) * z.get(r, 0);
         for (int r = 0; r < 3; r++)
            offToPack.add(r, 0, -component * u.get(r, k));
      }
   }

   /**
    * Max over the previous {@code causalWindow} ticks, never including the tick just recorded --
    * this is the whole of the circularity guarantee, so it is written as one small method rather
    * than spread through the caller. Tick 0 therefore emits zero, i.e. no inflation until there is
    * a past to read; the offline reference instead lets tick 0 see itself, which is a one-tick edge
    * artifact this side declines to reproduce.
    */
   private double trailingMaxExcludingNow(double[] ring, long currentTick)
   {
      long oldest = Math.max(0L, currentTick - causalWindow);
      double max = 0.0;
      for (long t = oldest; t < currentTick; t++)
         max = Math.max(max, ring[(int) (t % ring.length)]);
      return max;
   }
}

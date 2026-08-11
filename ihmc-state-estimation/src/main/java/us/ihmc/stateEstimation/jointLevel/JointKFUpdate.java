package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.chol.CholeskyDecompositionInner_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.dense.row.linsol.chol.LinearSolverChol_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.function.ToDoubleFunction;

/**
 * The joint-level KF's measurement updates over the joint states — the encoder position channel and the direct
 * joint-velocity channel, both of which observe the state through an identity Jacobian — plus the Joseph-form
 * core and its conditioning gates, which every channel shares (including
 * {@link JointKFBiasUpdate}'s stacked gyro block).
 *
 * @author Lucas Libshutz
 */
final class JointKFUpdate
{
   /**
    * Identifies the calling channel through {@link #josephUpdate}. This was a bare {@code String} tested four
    * DIFFERENT ways (two exact matches, two prefix matches), which is why it is an enum: each rule is now an
    * explicit field, so they cannot drift apart. {@link #TEST} takes the GYRO R-floor, so every unit test
    * driving the 3-arg overload is gated at half the gyro floor, not at an encoder floor.
    */
   enum Channel
   {
      /** Encoder position rows. Publishes the encoder NIS/innovation; floors S at the encoder R. */
      ENCODER("encoder", NisChannel.ENCODER, RFloor.ENCODER, false, true),
      /** Direct joint-velocity rows. Publishes the velocity NIS/innovation; floors S at the velocity R. */
      ENCODER_VELOCITY("encoderVelocity", NisChannel.VELOCITY, RFloor.VELOCITY, false, true),
      /** The stacked gyro block (pairs + stance anchors). Publishes the S diagnostics; floors S at the gyro R. */
      STACKED_GYRO("stackedGyroUpdate", NisChannel.NONE, RFloor.GYRO, true, false),
      /** The package-private 3-arg test overload. No NIS publication; gyro R-floor. */
      TEST("test", NisChannel.NONE, RFloor.GYRO, false, false);

      enum NisChannel {ENCODER, VELOCITY, NONE}

      enum RFloor {ENCODER, VELOCITY, GYRO}

      /** The label this channel logs as — identical to the pre-split strings, so log output is unchanged. */
      final String label;
      final NisChannel nisChannel;
      final RFloor rFloor;
      /** True for the block whose Cholesky S diagnostics are published every tick. */
      final boolean publishesSDiagnostics;
      /** True when the singular-innovation diagnostic should describe rows as encoder joints rather than gyro rows. */
      final boolean describeRowsAsEncoder;

      Channel(String label, NisChannel nisChannel, RFloor rFloor, boolean publishesSDiagnostics, boolean describeRowsAsEncoder)
      {
         this.label = label;
         this.nisChannel = nisChannel;
         this.rFloor = rFloor;
         this.publishesSDiagnostics = publishesSDiagnostics;
         this.describeRowsAsEncoder = describeRowsAsEncoder;
      }

      /** Maps the legacy label strings the package tests still pass to {@code josephUpdate}. */
      static Channel fromLabel(String label)
      {
         for (Channel c : values())
            if (c.label.equals(label))
               return c;
         return TEST;
      }
   }

   private final JointKFState state;
   private final JointKFParameters parameters;
   private final double dt;

   // Encoder position channel: H_enc = [I_n | 0 | 0].
   DMatrixRMaj Henc, Renc, zEnc;
   /** Wired per-joint encoder position measurement variance (rad^2), state order; the fallback where unwired. */
   private double[] encVarPerJoint;
   /** min_i encVarPerJoint[i]: the encoder-block S-pivot floor. A healthy S has every pivot >= its row's R_ii,
    *  so the collapsed-row gate must threshold on the SMALLEST wired variance. SEEDED to the fallback then
    *  min-reduced, not a plain min over the array: with every wired variance above the fallback the answer must
    *  stay the fallback, and with n == 0 the loop never runs. */
   private double encVarFloorMin;

   // Direct joint-velocity channel: the firmware reports a drive-side-filtered output velocity per joint, and
   // treating it as a measurement of q̇ pins the J·q̇ part of every pair-gyro row, making the IMU bias
   // DIFFERENCES inferable each tick instead of only through the process model. H_qd = [0 | I_n | 0].
   DMatrixRMaj Hqd, Rqd, zqd;
   /** Wired per-joint encoder velocity measurement variance ((rad/s)^2); the fallback where unwired. */
   private double[] qdMeasVarPerJoint;
   /** 1/omega_eff (s) of the measurement's low-pass cascade per joint; 0 => no lag inflation (unknown/sim). */
   private double[] invOmegaEffPerJoint;
   /** Last tick's measured q̇ (NaN before the first sample) and its smoothed slope. */
   private double[] prevZqd, qdSlewSmoothed;
   private double lagSlewSmoothingAlpha;
   /** min_i qdMeasVarPerJoint[i]: S-pivot floor for the velocity rows (lag inflation only ever raises R).
    *  Seeded to the fallback then min-reduced, same shape and for the same reason as encVarFloorMin. */
   private double qdVarFloorMin;

   // Joseph-form scratch, all pre-sized in the constructor at the widest measurement so no tick reallocates.
   private final DMatrixRMaj PHt = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj S = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj Sinv = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj K = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj nu = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj KR = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj IKH = new DMatrixRMaj(0, 0); // I-KH, dim x dim, Joseph form
   private final DMatrixRMaj Ptmp = new DMatrixRMaj(0, 0);
   /** Reused Cholesky solver for the innovation-covariance inverse. S is symmetric PD by construction, so a
    *  setA failure means non-PD => skip. Its factor L also gives the conditioning gate cheaply: S's pivots are
    *  L_ii^2, so cond(S) ~ (max L_ii / min L_ii)^2 with no eigendecomposition. */
   private LinearSolverChol_DDRM innovationSolver;
   private final DMatrixRMaj Lchol = new DMatrixRMaj(0, 0); // scratch for the innovation Cholesky factor L (k x k)

   // Rollback backups: a KF has no recovery once x/P go non-finite, so a single bad update is undone by
   // restoring the pre-update mean and covariance rather than latching NaN forever. The O(dim^2) copy is
   // negligible next to the O(dim^3) Joseph products already here.
   private final DMatrixRMaj xBackup = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj PBackup = new DMatrixRMaj(0, 0);

   /** Separate one-shot flag for the near-singular innovation-covariance diagnostic, so its detailed
    *  S-conditioning report is not swallowed by an unrelated earlier non-finite-input warning. */
   private boolean warnedSingularInnovation = false;

   private final YoInteger yoInnovationGateSkipCount;
   private final YoDouble yoCondSProxyLog10;
   private final YoDouble yoMinSDiag;
   private final YoDouble yoMaxSDiag;
   // Per-channel diagnostics, indexed by joint state index. NIS = nu_i^2 / S_ii; E[NIS] = 1 for a consistent
   // filter, so a per-joint mean far from 1 reads directly as "R_i is wrong by that factor". Published BEFORE
   // the conditioning gates, so a gated tick still shows the innovation that tripped it. encR is constant (the
   // wired variance, logged so a joint silently on the scalar default is visible); qdR is this tick's APPLIED
   // variance, floor plus lag inflation, so the inflation itself is visible in the log.
   private YoDouble[] yoEncNIS;
   private YoDouble[] yoEncR;
   // Signed innovations nu_i = z_i - (H x)_i. NIS squares the sign away; these keep it, which is what
   // discriminates lag-shaped from impact-shaped encoder innovations: corr(encInnov, qd_meas/(2*pi*25)) ~ +1
   // with slope ~1 implicates the 25 Hz drive-side position filter lag, while innovation concentrated in
   // <100 ms bursts at contact transitions implicates unmodeled impact acceleration vs Qa.
   private YoDouble[] yoEncInnov;
   private YoDouble[] yoQdInnov;
   private YoDouble[] yoQdNIS;
   private YoDouble[] yoQdR;
   /** Live kill switch, settable from SCS mid-run for the hardware A/B. */
   private YoBoolean yoUseDirectVelocity;

   JointKFUpdate(JointKFState state,
                 ToDoubleFunction<String> encoderPositionNoiseStd,
                 ToDoubleFunction<String> encoderVelocityNoiseStd,
                 ToDoubleFunction<String> jointVelocityMeasurementBreakFrequencyHz,
                 boolean useDirectVelocityMeasurement,
                 JointKFParameters parameters,
                 YoRegistry registry)
   {
      this.state = state;
      this.parameters = parameters;
      this.dt = state.dt;
      int n = state.numberOfJoints;

      // Per-joint encoder position variance, resolved once (lookup is by joint NAME; state order thereafter).
      // The S-pivot floor for the encoder block follows the smallest wired variance — see encVarFloorMin.
      encVarPerJoint = new double[n];
      double encoderVarFallback = parameters.encoderVar.getValue();
      int unwiredEncoderJoints = 0;
      for (var e : state.jointToIndex.entrySet())
      {
         double std = encoderPositionNoiseStd == null ? Double.NaN : encoderPositionNoiseStd.applyAsDouble(e.getKey().getName());
         if (Double.isFinite(std) && std > 0.0)
            encVarPerJoint[e.getValue()] = std * std;
         else
         {
            encVarPerJoint[e.getValue()] = encoderVarFallback;
            unwiredEncoderJoints++;
         }
      }
      encVarFloorMin = encoderVarFallback; // seed, then min-reduce (see the field doc)
      for (int i = 0; i < n; i++)
         encVarFloorMin = Math.min(encVarFloorMin, encVarPerJoint[i]);
      if (encoderPositionNoiseStd != null && unwiredEncoderJoints > 0)
         LogTools.warn("JointLevelKFPreFilter: " + unwiredEncoderJoints + " of " + n + " filtered joints have no wired "
               + "encoder position noise; they fall back to ENCODER_VAR = " + encoderVarFallback + " rad^2 — orders of magnitude "
               + "above the measured per-joint values, so those encoders will be badly under-trusted. Check jointKF_encR_<joint>.");

      // Direct-velocity wiring, resolved once by name. 1/omega_eff = 0 disables the lag inflation for that
      // joint, i.e. treats its measurement as instantaneous.
      qdMeasVarPerJoint = new double[n];
      invOmegaEffPerJoint = new double[n];
      prevZqd = new double[n];
      qdSlewSmoothed = new double[n];
      double sigmaQdUnfiltered = parameters.sigmaQdUnfiltered.getValue();
      double qdVarFallback = sigmaQdUnfiltered * sigmaQdUnfiltered;
      for (var e : state.jointToIndex.entrySet())
      {
         int idx = e.getValue();
         String name = e.getKey().getName();
         double std = encoderVelocityNoiseStd == null ? Double.NaN : encoderVelocityNoiseStd.applyAsDouble(name);
         qdMeasVarPerJoint[idx] = (Double.isFinite(std) && std > 0.0) ? std * std : qdVarFallback;
         double fc = jointVelocityMeasurementBreakFrequencyHz == null ? Double.NaN : jointVelocityMeasurementBreakFrequencyHz.applyAsDouble(name);
         invOmegaEffPerJoint[idx] = (Double.isFinite(fc) && fc > 0.0) ? 1.0 / (2.0 * Math.PI * fc) : 0.0;
         prevZqd[idx] = Double.NaN;
      }
      qdVarFloorMin = qdVarFallback; // seed, then min-reduce (see the field doc)
      for (int i = 0; i < n; i++)
         qdVarFloorMin = Math.min(qdVarFloorMin, qdMeasVarPerJoint[i]);
      lagSlewSmoothingAlpha = Math.exp(-2.0 * Math.PI * parameters.lagSlewSmoothingHz.getValue() * dt);
      if (useDirectVelocityMeasurement)
         LogTools.info("JointLevelKFPreFilter: direct joint-velocity measurement channel ENABLED over " + n
               + " joints (adaptive lag inflation on " + (int) java.util.Arrays.stream(invOmegaEffPerJoint).filter(v -> v > 0.0).count()
               + " of them; live kill switch: jointKFUseDirectVelocityMeasurement).");

      allocate();
      buildEncoderModel();
      buildDirectVelocityModel();

      yoInnovationGateSkipCount = new YoInteger("jointKFInnovationGateSkipCount", registry);
      yoCondSProxyLog10 = new YoDouble("jointKFCondSProxyLog10", registry);
      yoMinSDiag = new YoDouble("jointKFMinSDiag", registry);
      yoMaxSDiag = new YoDouble("jointKFMaxSDiag", registry);
      createMeasurementYoVariables(registry, useDirectVelocityMeasurement);
   }

   private void allocate()
   {
      int n = state.numberOfJoints;
      int dim = state.dim;
      // Widest measurement is either the n-joint encoder block or the 3(E+K_max) stacked gyro block; size all
      // the innovation scratch at that width so a full-anchor stacked update never reallocates on the estimator
      // thread. Both inputs come from JointKFState, never from the later-constructed JointKFBiasUpdate.
      int maxMeas = Math.max(n, state.maxStackRows);

      PHt.reshape(dim, maxMeas);
      K.reshape(dim, maxMeas);
      KR.reshape(dim, maxMeas);
      S.reshape(maxMeas, maxMeas);
      Sinv.reshape(maxMeas, maxMeas);
      nu.reshape(maxMeas, 1);
      Henc = new DMatrixRMaj(n, dim);
      Renc = new DMatrixRMaj(n, n);
      zEnc = new DMatrixRMaj(n, 1);
      zqd = new DMatrixRMaj(n, 1);

      // IKH MUST start at dim x dim: the first josephUpdate does setIdentity(IKH) BEFORE multAdd reshapes it, so
      // on a 0x0 IKH the identity is silently dropped and the first covariance update becomes -KH, not I-KH.
      Ptmp.reshape(dim, dim);
      IKH.reshape(dim, dim);
      xBackup.reshape(dim, 1);
      PBackup.reshape(dim, dim);

      // Warmed at maxMeas so per-tick setA/invert never allocate. Cholesky is the right factorization here:
      // setA returns false on a non-PD S, a strictly stronger reject than LU's exact-singular test.
      innovationSolver = new LinearSolverChol_DDRM(new CholeskyDecompositionInner_DDRM(true)); // lower-triangular L
      innovationSolver.setA(CommonOps_DDRM.identity(maxMeas));
      Lchol.reshape(maxMeas, maxMeas);
   }

   private void buildEncoderModel()
   {
      int n = state.numberOfJoints;
      Henc.zero();
      for (int i = 0; i < n; i++)
         Henc.set(i, i, 1.0);
      Renc.zero();
      for (int i = 0; i < n; i++)
         Renc.set(i, i, encVarPerJoint[i]);
   }

   /**
    * H_qd = [0 | I_n | 0] observes the q̇ block; R_qd starts at the measured per-joint floor. Unlike Renc it is
    * NOT constant — {@link #refreshDirectVelocityNoise()} re-diagonals it every tick before the update.
    */
   private void buildDirectVelocityModel()
   {
      int n = state.numberOfJoints;
      Hqd = new DMatrixRMaj(n, state.dim);
      for (int i = 0; i < n; i++)
         Hqd.set(i, n + i, 1.0);
      Rqd = new DMatrixRMaj(n, n);
      for (int i = 0; i < n; i++)
         Rqd.set(i, i, qdMeasVarPerJoint[i]);
   }

   private void createMeasurementYoVariables(YoRegistry registry, boolean useDirectVelocityMeasurement)
   {
      int n = state.numberOfJoints;
      yoEncNIS = new YoDouble[n];
      yoEncR = new YoDouble[n];
      yoEncInnov = new YoDouble[n];
      yoQdNIS = new YoDouble[n];
      yoQdR = new YoDouble[n];
      yoQdInnov = new YoDouble[n];
      yoUseDirectVelocity = new YoBoolean("jointKFUseDirectVelocityMeasurement", registry);
      yoUseDirectVelocity.set(useDirectVelocityMeasurement);
      for (var e : state.jointToIndex.entrySet())
      {
         int idx = e.getValue();
         String jointName = e.getKey().getName();
         yoEncNIS[idx] = new YoDouble("jointKF_encNIS_" + jointName, registry);
         yoEncNIS[idx].set(Double.NaN); // no encoder update has run yet
         yoEncInnov[idx] = new YoDouble("jointKF_encInnov_" + jointName, registry);
         yoEncInnov[idx].set(Double.NaN); // signed innovation (rad); no encoder update has run yet
         yoEncR[idx] = new YoDouble("jointKF_encR_" + jointName, registry);
         yoEncR[idx].set(encVarPerJoint[idx]); // constant: the wired measurement variance (rad^2)
         yoQdNIS[idx] = new YoDouble("jointKF_qdNIS_" + jointName, registry);
         yoQdNIS[idx].set(Double.NaN); // no direct-velocity update has run yet
         yoQdInnov[idx] = new YoDouble("jointKF_qdInnov_" + jointName, registry);
         yoQdInnov[idx].set(Double.NaN); // signed innovation (rad/s); no direct-velocity update has run yet
         yoQdR[idx] = new YoDouble("jointKF_qdR_" + jointName, registry);
         yoQdR[idx].set(qdMeasVarPerJoint[idx]); // per-tick: floor + adaptive lag inflation
      }
   }

   /** Live kill switch for the direct-velocity channel (flippable from SCS mid-run for a hardware A/B). */
   boolean isDirectVelocityEnabled()
   {
      return yoUseDirectVelocity != null && yoUseDirectVelocity.getValue();
   }

   /**
    * Adaptive R for the direct-velocity channel. For a first-order low-pass y = LPF_ω(u) the identity
    * u − y = ẏ/ω is EXACT, so the instantaneous lag error is the measured signal's own slope over the corner
    * (stages cascade as 1/ω_eff = Σ 1/ω_stage), giving
    * <pre>   R_ii(t) = σ_i² + (d̂_i(t) / ω_eff,i)² ,   d̂ = low-passed (z_k − z_{k−1})/dt</pre>
    * At quiet stance d̂ → 0 and the channel runs at its noise floor — full sharpness exactly in the quasi-static
    * regime the gyro-bias split needs it — while each joint softens during fast swings by its own lag error. A
    * static inflation would instead detune the channel permanently.
    */
   void refreshDirectVelocityNoise()
   {
      for (int i = 0; i < state.numberOfJoints; i++)
      {
         double z = zqd.get(i, 0);
         if (Double.isFinite(prevZqd[i]))
         {
            double slew = (z - prevZqd[i]) / dt;
            qdSlewSmoothed[i] = lagSlewSmoothingAlpha * qdSlewSmoothed[i] + (1.0 - lagSlewSmoothingAlpha) * slew;
         }
         prevZqd[i] = z;
         double lagError = qdSlewSmoothed[i] * invOmegaEffPerJoint[i];
         double variance = qdMeasVarPerJoint[i] + lagError * lagError;
         Rqd.set(i, i, variance);
         if (yoQdR != null)
            yoQdR[i].set(variance);
      }
   }

   /** Loads a measured q̇ vector (filter state order) into zqd so tests can drive refreshDirectVelocityNoise(). */
   void setDirectVelocityMeasurementForTest(double[] qdMeasured)
   {
      for (int i = 0; i < state.numberOfJoints; i++)
         zqd.set(i, 0, qdMeasured[i]);
   }

   // ================================ KF core ================================

   /** Backward-compatible overload for the package-private unit tests: uses the TEST channel. */
   void josephUpdate(DMatrixRMaj Hm, DMatrixRMaj zm, DMatrixRMaj Rm)
   {
      josephUpdate(Hm, zm, Rm, Channel.TEST);
   }

   /** Label-string overload retained for the package tests, which pass "encoder" / "encoderVelocity". */
   void josephUpdate(DMatrixRMaj Hm, DMatrixRMaj zm, DMatrixRMaj Rm, String label)
   {
      josephUpdate(Hm, zm, Rm, Channel.fromLabel(label));
   }

   /**
    * Joseph-form measurement update, hardened against latching NaN with two guards over a textbook update: a
    * non-finite S^-1 skips the update BEFORE x or P are touched, and a completed update that still leaves them
    * non-finite is rolled back from the backups. Both leave the filter in its prior valid state.
    */
   void josephUpdate(DMatrixRMaj Hm, DMatrixRMaj zm, DMatrixRMaj Rm, Channel channel)
   {
      int dim = state.dim;
      int k = Hm.getNumRows();
      PHt.reshape(dim, k);
      S.reshape(k, k);
      Sinv.reshape(k, k);
      nu.reshape(k, 1);

      CommonOps_DDRM.multTransB(state.P, Hm, PHt);
      CommonOps_DDRM.mult(Hm, PHt, S);
      CommonOps_DDRM.addEquals(S, Rm); // S = H P H^T + R; the +R was once missing, which made the gain
                                       // over-confident and could grow the covariance
      // Symmetrize first: Cholesky assumes exact symmetry and S is symmetric only to round-off. The factor
      // diagonal then gives both gates for free (pivots are L_ii^2). A finite-but-ill-conditioned S inverts to a
      // huge gain that the Joseph K R K^T loop squares each tick — the divergence the old LU/isFinite guards
      // were blind to. The floor gate catches the dual pathology: a pivot below half the applicable noise floor
      // is algebraically impossible for a healthy S, so it signals a collapsed (zero-Sigma) row.
      JointLevelKFPreFilter.symmetrize(S);
      // nu = z - H x, computed BEFORE the gates: it depends on neither the factorization nor the gain, and S's
      // diagonal must be read here anyway since setA may decompose in place.
      CommonOps_DDRM.mult(Hm, state.x, nu);
      CommonOps_DDRM.changeSign(nu);
      CommonOps_DDRM.addEquals(nu, zm);
      // Per-joint NIS for the two identity-block channels (row i IS joint i in state order). The channels are
      // DISTINCT and must never cross-publish — an enum field, not a string prefix, is what keeps that true.
      if (channel.nisChannel == Channel.NisChannel.ENCODER && yoEncNIS != null)
      {
         for (int i = 0; i < k; i++)
         {
            yoEncNIS[i].set(nu.get(i, 0) * nu.get(i, 0) / S.get(i, i));
            yoEncInnov[i].set(nu.get(i, 0)); // rad
         }
      }
      else if (channel.nisChannel == Channel.NisChannel.VELOCITY && yoQdNIS != null)
      {
         for (int i = 0; i < k; i++)
         {
            yoQdNIS[i].set(nu.get(i, 0) * nu.get(i, 0) / S.get(i, i));
            yoQdInnov[i].set(nu.get(i, 0)); // rad/s
         }
      }
      if (!innovationSolver.setA(S))
      {
         warnSingularInnovationOnce(channel, "S is not positive definite (Cholesky factorization rejected it)", Hm, Rm);
         return;
      }
      Lchol.reshape(k, k); // shrink into the capacity pre-reserved in allocate() (no realloc); getT needs an exact k x k target
      innovationSolver.getDecomposition().getT(Lchol);
      double minLii = Double.POSITIVE_INFINITY, maxLii = 0.0;
      for (int i = 0; i < k; i++)
      {
         double lii = Lchol.get(i, i);
         if (lii < minLii) minLii = lii;
         if (lii > maxLii) maxLii = lii;
      }
      double minPivot = minLii * minLii;                 // ~ lambda_min(S)
      double maxPivot = maxLii * maxLii;                 // ~ lambda_max(S)
      double condProxy = minLii > 0.0 ? (maxLii / minLii) * (maxLii / minLii) : Double.POSITIVE_INFINITY;
      if (channel.publishesSDiagnostics) // publish the gyro-block S diagnostics every tick (cheap)
      {
         yoCondSProxyLog10.set(Math.log10(condProxy));
         yoMinSDiag.set(minPivot);
         yoMaxSDiag.set(maxPivot);
      }
      double condSMax = parameters.condSMax.getValue();
      if (condProxy > condSMax)
      {
         yoInnovationGateSkipCount.increment();
         warnSingularInnovationOnce(channel, String.format("cond(S) proxy %.3e exceeds COND_S_MAX %.1e; whole block gated", condProxy, condSMax), Hm, Rm);
         return;
      }
      // Channel-specific S-pivot floor: the block's min wired R_ii. Lag inflation only ever RAISES the velocity
      // rows' R, so the boot-time min stays a valid lower bound.
      double rFloor;
      switch (channel.rFloor)
      {
         case ENCODER -> rFloor = encVarFloorMin;
         case VELOCITY -> rFloor = qdVarFloorMin;
         default -> rFloor = parameters.sigmaGyroFloor.getValue();
      }
      if (minPivot < 0.5 * rFloor)
      {
         yoInnovationGateSkipCount.increment();
         warnSingularInnovationOnce(channel, String.format("min S pivot %.3e below half the R floor %.3e (collapsed row); block gated", minPivot, rFloor), Hm, Rm);
         return;
      }
      innovationSolver.invert(Sinv);
      // A PD, well-conditioned S still gets a residual-overflow guard before x/P are touched.
      if (JointLevelKFPreFilter.containsNonFinite(Sinv))
      {
         warnSingularInnovationOnce(channel, "S^-1 is non-finite (S is near-singular and its inverse overflowed)", Hm, Rm);
         return;
      }

      // Snapshot for rollback: everything below writes x then P in place.
      xBackup.set(state.x);
      PBackup.set(state.P);

      K.reshape(dim, k);
      CommonOps_DDRM.mult(PHt, Sinv, K);
      CommonOps_DDRM.multAdd(K, nu, state.x); // nu = z - H x was computed above, before the gates

      // Full Joseph form update
      CommonOps_DDRM.setIdentity(IKH);
      CommonOps_DDRM.multAdd(-1.0, K, Hm, IKH); // I - KH
      CommonOps_DDRM.mult(IKH, state.P, Ptmp);
      CommonOps_DDRM.multTransB(Ptmp, IKH, state.P);
      KR.reshape(dim, k);
      CommonOps_DDRM.mult(K, Rm, KR);
      CommonOps_DDRM.multAddTransB(KR, K, state.P); // + K R K^T

      if (JointLevelKFPreFilter.containsNonFinite(state.x) || JointLevelKFPreFilter.containsNonFinite(state.P))
      {
         state.x.set(xBackup);
         state.P.set(PBackup);
         if (!state.warnedNonFiniteInput)
            state.warnNonFiniteInputOnce("the " + channel.label + " update produced a non-finite state; rolled back to the prior estimate");
      }
   }

   // ================================ Singular-innovation diagnostics ================================

   /**
    * One-shot diagnostic for a near-singular S, which forces its update to be skipped — a hidden way the filter
    * keeps diverging even after the Qa tripwire binds. Names WHICH physical measurement is degenerate: S is
    * symmetric PSD, so the eigenvector of its smallest eigenvalue is the direction with almost no innovation
    * variance. The eigen/string work runs only on this occurrence, never on the healthy hot path.
    */
   private void warnSingularInnovationOnce(Channel channel, String reason, DMatrixRMaj Hm, DMatrixRMaj Rm)
   {
      if (warnedSingularInnovation)
         return;
      warnedSingularInnovation = true;
      LogTools.warn(describeSingularInnovation(channel, reason, Hm, Rm) + " Reported once.");
   }

   /** Label-string overload retained for the package tests. */
   String describeSingularInnovation(String label, String reason, DMatrixRMaj Hm, DMatrixRMaj Rm)
   {
      return describeSingularInnovation(Channel.fromLabel(label), reason, Hm, Rm);
   }

   /**
    * Builds the diagnostic string. Package-private so a test can assert the row -> physical-element mapping
    * directly instead of scraping log output. S is recomputed from {@code Hm} because the solver's
    * {@code setA} may have overwritten the shared S buffer in place with its factors.
    */
   String describeSingularInnovation(Channel channel, String reason, DMatrixRMaj Hm, DMatrixRMaj Rm)
   {
      int dim = state.dim;
      int k = Hm.getNumRows();
      // Recompute S = H P H^T + R fresh (setA above may have decomposed the field S in place). Rare path — ok to allocate.
      DMatrixRMaj phT = new DMatrixRMaj(dim, k);
      DMatrixRMaj Sfresh = new DMatrixRMaj(k, k);
      CommonOps_DDRM.multTransB(state.P, Hm, phT);
      CommonOps_DDRM.mult(Hm, phT, Sfresh);
      CommonOps_DDRM.addEquals(Sfresh, Rm);

      StringBuilder msg = new StringBuilder();
      msg.append("JointLevelKFPreFilter: near-singular innovation covariance in the ").append(channel.label)
         .append(" update — ").append(reason).append(", so this measurement was skipped this tick. ");

      EigenDecomposition_F64<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(k, true, true); // symmetric, with eigenvectors
      if (eig.decompose(Sfresh))
      {
         int minIdx = 0, maxIdx = 0;
         double minEig = Double.POSITIVE_INFINITY, maxEig = Double.NEGATIVE_INFINITY;
         for (int i = 0; i < k; i++)
         {
            double lambda = eig.getEigenvalue(i).getReal();
            if (lambda < minEig) { minEig = lambda; minIdx = i; }
            if (lambda > maxEig) { maxEig = lambda; maxIdx = i; }
         }
         double cond = minEig != 0.0 ? Math.abs(maxEig / minEig) : Double.POSITIVE_INFINITY;
         // Report BOTH ends: when the pathology is a lambda_MAX blow-up (P diverging), the SMALLEST eigenvector
         // points at the HEALTHIEST rows, so naming only the min side mis-blames the stance-leg encoders.
         final double SANE_INNOVATION_SCALE = 1.0e6;
         boolean maxSidePathological = maxEig > SANE_INNOVATION_SCALE;
         msg.append(String.format("cond(S)=%.3e, lambda_min=%.3e, lambda_max=%.3e — the %s side is pathological. ",
                                  cond, minEig, maxEig,
                                  maxSidePathological ? "MAX (lambda_max above a sane innovation scale ~1e6; P has diverged)"
                                                      : "MIN (lambda_min collapsed toward 0; a row lost its noise floor)"));
         msg.append("min-side rows (near-null measurement directions): ");
         appendDominantRows(msg, eig.getEigenVector(minIdx), k, channel, Hm);
         msg.append(". max-side rows (P-blowup directions): ");
         appendDominantRows(msg, eig.getEigenVector(maxIdx), k, channel, Hm);
      }
      else
      {
         int minRow = 0;
         double minDiag = Double.POSITIVE_INFINITY;
         for (int i = 0; i < k; i++)
            if (Sfresh.get(i, i) < minDiag) { minDiag = Sfresh.get(i, i); minRow = i; }
         msg.append(String.format("(eigen-decomposition failed; smallest S diagonal %.3e at %s)",
                                  minDiag, describeMeasurementRow(minRow, channel, Hm)));
      }
      msg.append(" This is where the innovation ill-conditioning lives.");
      return msg.toString();
   }

   /** The up-to-4 rows carrying the most weight in eigenvector {@code v}, as "|weight| -> physical element". */
   private void appendDominantRows(StringBuilder msg, DMatrixRMaj v, int k, Channel channel, DMatrixRMaj Hm)
   {
      if (v == null)
      {
         msg.append("(eigenvector unavailable)");
         return;
      }
      boolean[] used = new boolean[k];
      int reported = 0;
      for (int rank = 0; rank < 4; rank++)
      {
         int best = -1;
         double bestAbs = -1.0;
         for (int i = 0; i < k; i++)
         {
            if (used[i])
               continue;
            double a = Math.abs(v.get(i));
            if (a > bestAbs) { bestAbs = a; best = i; }
         }
         if (best < 0 || bestAbs < 1.0e-3)
            break;
         used[best] = true;
         if (reported > 0)
            msg.append("; ");
         msg.append(String.format("%.2f -> %s", bestAbs, describeMeasurementRow(best, channel, Hm)));
         reported++;
      }
      if (reported == 0)
         msg.append("(spread across the whole block; no single row dominates)");
   }

   /**
    * Maps a measurement row to the physical element it observes. Encoder rows go through {@code Hm} (the
    * observed joint is the column with the unit entry), so this stays correct even if the encoder update is
    * later gated to a joint subset. Gyro rows follow the stacked layout: pair e, then the active anchors.
    */
   private String describeMeasurementRow(int row, Channel channel, DMatrixRMaj Hm)
   {
      if (channel.describeRowsAsEncoder)
      {
         int col = -1;
         for (int c = 0; c < Hm.getNumCols(); c++)
            if (Hm.get(row, c) != 0.0) { col = c; break; }
         return "encoder q of joint " + ((col >= 0 && col < state.numberOfJoints) ? state.jointNameByStateIndex(col) : "?");
      }
      int block = row / 3;
      int axis = row % 3;
      char ax = axis == 0 ? 'x' : axis == 1 ? 'y' : 'z';
      if (block < state.numberOfIMUPairs)
      {
         JointKFState.Pair p = state.pairs.get(block);
         return "gyro pair " + block + " (" + p.parent.getSensorName() + "->" + p.child.getSensorName()
                + "), axis " + ax + ", chain=[" + JointKFState.jointNamesOf(p.chainJoints) + "]";
      }
      JointKFState.FootAnchor fa = nthActiveAnchor(block - state.numberOfIMUPairs);
      if (fa != null)
         return "stance anchor (foot " + fa.foot.getName() + "), axis " + ax + ", leg=[" + JointKFState.jointNamesOf(fa.legJoints) + "]";
      return "gyro row " + row + " (unmapped)";
   }

   /** The {@code activeIdx}-th anchor with {@code active == true}, in footAnchors order (matches the stacked-row layout). */
   private JointKFState.FootAnchor nthActiveAnchor(int activeIdx)
   {
      int count = 0;
      for (int i = 0; i < state.footAnchors.size(); i++)
      {
         JointKFState.FootAnchor fa = state.footAnchors.get(i);
         if (fa.active)
         {
            if (count == activeIdx)
               return fa;
            count++;
         }
      }
      return null;
   }
}

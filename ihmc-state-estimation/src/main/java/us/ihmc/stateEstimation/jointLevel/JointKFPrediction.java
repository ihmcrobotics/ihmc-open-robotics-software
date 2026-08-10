package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

/**
 * The joint-level KF's time update: the constant transition F, the process noise Q, and the whole
 * Schur-complement mass-matrix path that makes Q configuration-dependent.
 *
 * <p><b>Process noise (SPEC §3.2).</b> The unmodeled joint torque w_tau maps to joint acceleration through the
 * FLOATING-BASE dynamics, not the locked-base map: eliminating the (unforced) nuisance acceleration from the
 * perturbed EOM gives delta_qddot = Lambda^-1 w_tau, Lambda = M_jj - M_jN M_NN^-1 M_Nj the Schur complement of
 * the nuisance block. Hence Qa = sigma_tau^2 Lambda^-2.</p>
 *
 * <p>"Nuisance" = the 6-DoF floating base (SPEC §3.2) plus any UNFILTERED joints that lie on the tree path
 * between the base and a filtered joint ("gap" joints). In the real robot the base IMU is on the base link
 * and every joint on a pair/anchor path is itself filtered, so there are NO gap joints and this is exactly
 * the SPEC's Lambda = M_jj - M_jb M_bb^-1 M_bj over the plain 6-DoF base. The gap term is a topology
 * generalization (needed because the composite-rigid-body calculator prunes the subtree below any ignored
 * joint, so an unfiltered joint above a filtered one cannot be silently locked): such gap joints are
 * instead marginalized (treated as free) alongside the base — the conservative choice (more recoil => more
 * process noise, consistent with §3.2's free-flyer upper bound). Genuinely OFF-path joints stay ignored and
 * their inertia is composited into the adjacent link by the calculator (considerIgnoredSubtreesInertia).</p>
 *
 * <p>The mass matrix M is built over {base} ∪ {joints spanning base->filtered}. All columns (nuisance and each
 * filtered joint) are resolved through the calculator's index provider — never assume ordering. Null
 * calculator (no robot model, or no 6-DoF floating base found) => the constant scalar-CWNA diagonal fallback.
 * NOTE: CompositeRigidBodyMassMatrixCalculator is correct here. DynamicsMatrixCalculator merely wraps this
 * same calculator (it holds one internally, built over the whole-body-control toolbox) and computes the
 * identical M(q) — switching changes nothing numerically. Process-noise magnitude is governed by the per-joint
 * sigma_tau and the QA_MAX conditioning tripwire, not by the choice of calculator.</p>
 *
 * @author Lucas Libshutz
 */
final class JointKFPrediction
{
   private final JointKFState state;
   private final JointKFParameters parameters;
   private final double dt;

   final DMatrixRMaj F = new DMatrixRMaj(0, 0); // dim x dim transition matrix
   final DMatrixRMaj Q = new DMatrixRMaj(0, 0); // dim x dim process noise
   private final DMatrixRMaj xtmp = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj Ptmp = new DMatrixRMaj(0, 0);
   /** 3x3 scratch for each IMU's angular-velocity BIAS PROCESS-noise covariance (the random-walk intensity).
    *  Deliberately NOT shared with the gyro MEASUREMENT-noise scratch in JointKFBiasUpdate: conflating those two
    *  quantities is a documented Rev. 1 over-confidence bug class, and separate buffers make that impossible. */
   private final DMatrixRMaj Rimu = new DMatrixRMaj(3, 3);

   private CompositeRigidBodyMassMatrixCalculator massMatrixCalculator;
   private int numberOfNuisanceDOF;         // 6 (base) + number of gap joints; the marginalized block width N
   private int[] massMatrixNuisanceColumns; // nuisance DoF columns of the full mass matrix (length N)
   private int[] massMatrixColumn;          // filter joint state index -> DoF column in the calculator's mass matrix
   private final DMatrixRMaj MNN = new DMatrixRMaj(0, 0);       // nuisance block, N x N
   private final DMatrixRMaj MNf = new DMatrixRMaj(0, 0);       // nuisance-filtered coupling, N x n (= M_jN^T)
   private final DMatrixRMaj Mff = new DMatrixRMaj(0, 0);       // filtered block, n x n
   private final DMatrixRMaj MNNInvMNf = new DMatrixRMaj(0, 0); // X = M_NN^-1 M_Nf, N x n
   private final DMatrixRMaj Lambda = new DMatrixRMaj(0, 0);    // Schur complement Lambda_eff (rotor diag added in place), n x n
   private final DMatrixRMaj LambdaInv = new DMatrixRMaj(0, 0); // Lambda_eff^-1, n x n
   private final DMatrixRMaj Ytau = new DMatrixRMaj(0, 0);      // Lambda_eff^-1 with column j scaled by sigma_tau,j, n x n
   private final DMatrixRMaj Qa = new DMatrixRMaj(0, 0);        // Qa = Ytau Ytau^T (PSD, symmetric BY CONSTRUCTION), n x n
   private LinearSolverDense<DMatrixRMaj> nuisanceMassMatrixSolver; // Cholesky over M_NN, solves M_NN X = M_Nf
   private LinearSolverDense<DMatrixRMaj> schurSolver;               // Cholesky over Lambda_eff, inverts it

   // Reflected rotor inertia diagonals (Part B item 1), in filter/nuisance order. The FILTERED-joint arrays are
   // refreshed every predict from the LIVE per-joint YoVariables below (see refreshRotorInertiaAndSigmaTau); the
   // nuisance one is fixed at construction (a gap joint has no tunable of its own — none exist on Alex).
   private double[] rotorInertiaDiag;         // length n, added to Lambda's diagonal (filter joint state order)
   private double[] nuisanceRotorInertiaDiag; // length numNuisanceDoF: 0 on base rows, rotor inertia on gap-joint rows
   private double[] sigmaTauPerJoint;         // length n, sigma_tau,i = alpha_i * tau_max,i (Part B item 3)
   // LIVE per-joint tuning, indexed by joint state index. Published as jointKFParam_alpha_<joint> and
   // jointKFParam_rotorInertia_<joint> so the documented ALPHA equalization runs in SCS without a rebuild.
   private YoDouble[] yoAlpha;
   private YoDouble[] yoRotorInertia;
   private double[] tauMaxPerJoint;      // length n, effort limit (N*m); NaN where the fallback applies
   private boolean[] sigmaTauIsFallback; // length n, true => no finite effort limit, use sigmaTauFallback
   private double sigmaTauFallback;      // boot-time read of jointKFParam_sigmaTau

   private boolean warnedMassMatrixFailure = false;
   private boolean warnedMassMatrixConditioningCap = false;

   /** Aggregate QA_MAX tripwire counter. Created FIRST, before the initial Q build, so it keeps the pre-split
    *  behavior of latching a construction-time bind (the per-joint arrays below deliberately do not — see the
    *  constructor). */
   private final YoInteger yoQaCapWouldBindCount;
   // Per-joint process-noise diagnostics (Part B item 2, extended for the ALPHA retune). yoQaDiag[i] is
   // diag(Qa)_i — joint i's acceleration process-noise VARIANCE ((rad/s^2)^2) this tick, the quantity the QA_MAX
   // tripwire thresholds. yoQaCapBindCount[i] counts the ticks joint i was the argmax that exceeded QA_MAX. The
   // aggregate jointKFQaCapWouldBindCount says the cap binds but not WHICH joint, so it cannot drive the per-joint
   // ALPHA retune (equalize sqrt(diag(Qa)) across joints -> alpha_i = sigma*_qdd / (|Lambda_eff^-1|_ii * tau_max,i));
   // these per-joint reads are the measurement that closes it. Indexed by joint state index.
   private YoDouble[] yoQaDiag;
   private YoInteger[] yoQaCapBindCount;

   JointKFPrediction(JointKFState state, RigidBodyBasics rootBody, JointKFParameters parameters, YoRegistry registry)
   {
      this.state = state;
      this.parameters = parameters;
      this.dt = state.dt;

      yoQaCapWouldBindCount = new YoInteger("jointKFQaCapWouldBindCount", registry);

      setupMassMatrix(rootBody, registry);
      // ORDER IS LOAD-BEARING: these two fill rotorInertiaDiag / sigmaTauPerJoint and the Qa telemetry arrays,
      // all of which the first buildProcessNoise below reads through updateProcessNoiseFromMassMatrix.
      // Pre-split, the Qa YoVariables were created after that first build and the update carried null guards for
      // them; creating them here instead means the boot-time Qa is published like every later tick's, and it
      // makes the per-joint jointKF_QaCapBind_<joint>_count consistent with the aggregate
      // jointKFQaCapWouldBindCount, which always latched a construction-time bind.
      createRotorInertiaAndSigmaTauParameters(registry);
      createQaYoVariables(registry);
      allocate();
      buildConstantTransition();
      buildProcessNoise();
   }

   private void setupMassMatrix(RigidBodyBasics rootBody, YoRegistry registry)
   {
      int n = state.n;
      // Schur-complement process noise (SPEC §3.2): the calculator is built over the FLOATING BASE plus the
      // filtered joints of the LIVE estimator model (the same joint objects the Jacobians read), so its M(q)
      // tracks the consumer-updated configuration each tick with no extra bookkeeping. Unlike Rev. 1 (joints
      // only), the floating joint MUST be included so the 6x6 base block M_bb and the base-joint coupling M_bj
      // are available for the Schur complement. Column mappings (base and each joint) are resolved through the
      // index provider rather than assumed from list order.
      if (rootBody != null && n > 0)
      {
         List<OneDoFJointBasics> massMatrixJoints = new ArrayList<>(state.jointToIndex.keySet());
         // The filtered joints ARE joints of the passed-in robot model; the model root is only used as a
         // sanity check here.
         RigidBodyBasics treeRoot = MultiBodySystemTools.getRootBody(massMatrixJoints.get(0).getPredecessor());
         if (treeRoot != rootBody)
            LogTools.warn("The provided robot model root '" + rootBody.getName() + "' is not the root of the filtered joints' tree;"
                          + " M(q) is computed on the joints' own tree.");

         // The base joint is the 6-DoF free-flyer at the root of the filtered joints' tree (elevator's child).
         JointReadOnly baseJoint = findFloatingBaseJoint(treeRoot);
         if (baseJoint == null)
         {
            // No floating base (e.g. a fixed-base model): the Schur complement is undefined, so degrade to the
            // scalar-CWNA fallback rather than silently reverting to the locked-base map this revision removes.
            LogTools.warn("Joint-level KF process noise: no 6-DoF floating base joint found at the tree root; "
                          + "falling back to scalar CWNA (the Schur-complement path needs a floating base — SPEC §3.2).");
         }
         else
         {
            // Joints to CONSIDER = base joint + the joints spanning base->filtered (every filtered joint plus any
            // unfiltered "gap" joints above it). The gap joints MUST be considered, not ignored: the calculator
            // prunes the whole subtree below an ignored joint, which would zero the filtered joints' inertia if an
            // unfiltered joint sat above them. Genuinely off-path joints are neither filtered nor spanning, so they
            // fall in getJointsToIgnore() and their inertia is composited (considerIgnoredSubtreesInertia).
            LinkedHashSet<JointReadOnly> spanningJoints = collectSpanningJoints(baseJoint, massMatrixJoints);
            List<JointReadOnly> jointsToConsider = new ArrayList<>();
            jointsToConsider.add(baseJoint);
            jointsToConsider.addAll(spanningJoints);
            MultiBodySystemReadOnly massMatrixInput = MultiBodySystemReadOnly.toMultiBodySystemInput(jointsToConsider);
            massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(massMatrixInput);

            var indexProvider = massMatrixInput.getJointMatrixIndexProvider();
            // Filtered-joint columns, in filter state order.
            massMatrixColumn = new int[n];
            for (var e : state.jointToIndex.entrySet())
               massMatrixColumn[e.getValue()] = indexProvider.getJointDoFIndices(e.getKey())[0];
            // Nuisance columns to marginalize = base 6-DoF columns + each gap joint's column (a gap joint is a
            // spanning joint that is not itself filtered). These are exactly the considered DoFs that are not
            // filtered joints.
            int[] baseColumns = indexProvider.getJointDoFIndices(baseJoint);
            List<Integer> nuisanceColumns = new ArrayList<>();
            // Parallel rotor-inertia diagonal for the nuisance block (Part B item 1): 0 on the 6 base DoF (the
            // free base carries no reflected rotor inertia), the joint's reflected rotor inertia on each gap
            // joint (so a marginalized unfiltered joint still floors M_NN). None on Alex today; kept general.
            List<Double> nuisanceRotor = new ArrayList<>();
            for (int c : baseColumns)
            {
               nuisanceColumns.add(c);
               nuisanceRotor.add(0.0);
            }
            int gapJoints = 0;
            for (JointReadOnly spanningJoint : spanningJoints)
            {
               if (!state.jointToIndex.containsKey(spanningJoint)) // unfiltered => gap joint => marginalize it
               {
                  nuisanceColumns.add(indexProvider.getJointDoFIndices(spanningJoint)[0]);
                  // Gap joints take the table value or the conservative default, silently — unlike a filtered
                  // joint, whose unmatched case is worth a warn (see createRotorInertiaAndSigmaTauParameters).
                  nuisanceRotor.add(JointKFParameters.reflectedRotorInertiaForNameOrDefault(spanningJoint.getName()));
                  gapJoints++;
               }
            }
            massMatrixNuisanceColumns = new int[nuisanceColumns.size()];
            nuisanceRotorInertiaDiag = new double[nuisanceColumns.size()];
            for (int i = 0; i < massMatrixNuisanceColumns.length; i++)
            {
               massMatrixNuisanceColumns[i] = nuisanceColumns.get(i);
               nuisanceRotorInertiaDiag[i] = nuisanceRotor.get(i);
            }
            numberOfNuisanceDOF = massMatrixNuisanceColumns.length; // 6 + gap joints

            LogTools.info("Joint-level KF process noise: Schur-complement path (sigma_tau = " + parameters.sigmaTau.getValue() + " N*m) over "
                          + n + " joints, marginalizing a " + numberOfNuisanceDOF + "-DoF nuisance block (6-DoF floating base"
                          + (gapJoints > 0 ? " + " + gapJoints + " unfiltered gap joint(s))." : ")."));
         }
      }
      else
      {
         LogTools.info("Joint-level KF process noise: scalar CWNA fallback (no robot model provided).");
      }
   }

   /**
    * Publishes the per-joint reflected rotor inertia (Part B item 1) and the per-joint alpha that sets the
    * unmodeled-torque STD sigma_tau,i = alpha_i * tau_max,i (Part B item 3) as LIVE YoVariables, in filter joint
    * state order, seeded from the {@link JointKFParameters} tables. Any joint with no rotor-table match is
    * seeded with the conservative default floor (warn); any joint with no finite positive effort limit is
    * pinned to the scalar sigmaTau fallback (warn) and its alpha is then ignored.
    *
    * <p>Both are re-read every tick by {@link #refreshRotorInertiaAndSigmaTau()}, so the ALPHA equalization
    * documented in {@link JointKFParameters} can be iterated live in SCS against jointKF_QaDiag_&lt;joint&gt;
    * instead of through a recompile.</p>
    */
   private void createRotorInertiaAndSigmaTauParameters(YoRegistry registry)
   {
      int n = state.n;
      rotorInertiaDiag = new double[n];
      sigmaTauPerJoint = new double[n];
      yoRotorInertia = new YoDouble[n];
      yoAlpha = new YoDouble[n];
      tauMaxPerJoint = new double[n];
      sigmaTauIsFallback = new boolean[n];
      sigmaTauFallback = parameters.sigmaTau.getValue();

      for (var e : state.jointToIndex.entrySet())
      {
         OneDoFJointBasics joint = e.getKey();
         int idx = e.getValue();
         String jointName = joint.getName();

         if (JointKFParameters.isRotorInertiaUnmatched(jointName))
            LogTools.warn("JointLevelKFPreFilter: no reflected-rotor-inertia table entry for filtered joint '"
                          + jointName + "'; applying the conservative default floor "
                          + parameters.rotorInertiaDefault.getValue() + " kg*m^2.");
         yoRotorInertia[idx] = new YoDouble("jointKFParam_rotorInertia_" + jointName,
                                            "LIVE: reflected rotor inertia n^2 J_rotor (kg*m^2) added to Lambda's diagonal "
                                            + "before inversion. Floors lambda_min(Lambda_eff), so lowering it inflates Qa.",
                                            registry);
         yoRotorInertia[idx].set(JointKFParameters.reflectedRotorInertiaForNameOrDefault(jointName));

         double tauMax = joint.getEffortLimitUpper();
         sigmaTauIsFallback[idx] = !Double.isFinite(tauMax) || tauMax <= 0.0;
         if (sigmaTauIsFallback[idx])
         {
            LogTools.warn("JointLevelKFPreFilter: filtered joint '" + jointName + "' has no finite positive effort"
                          + " limit (" + tauMax + "); falling back to the scalar SIGMA_TAU = " + sigmaTauFallback
                          + " N*m for its sigma_tau.");
            tauMaxPerJoint[idx] = Double.NaN; // unused on the fallback path; NaN so a regression is loud
         }
         else
         {
            tauMaxPerJoint[idx] = tauMax;
         }
         yoAlpha[idx] = new YoDouble("jointKFParam_alpha_" + jointName,
                                     "LIVE: fraction of this joint's torque capacity treated as unmodeled; "
                                     + "sigma_tau = alpha * tau_max. Retune against jointKF_QaDiag_" + jointName
                                     + " to equalize sqrt(diag(Qa)) at jointKFParam_targetQddStd."
                                     + (sigmaTauIsFallback[idx] ? " IGNORED for this joint: no finite effort limit, so "
                                                                  + "jointKFParam_sigmaTau is used instead." : ""),
                                     registry);
         yoAlpha[idx].set(JointKFParameters.alphaForName(jointName));
      }

      refreshRotorInertiaAndSigmaTau();
   }

   private void createQaYoVariables(YoRegistry registry)
   {
      int n = state.n;
      yoQaDiag = new YoDouble[n];
      yoQaCapBindCount = new YoInteger[n];
      for (var e : state.jointToIndex.entrySet())
      {
         int idx = e.getValue();
         String jointName = e.getKey().getName();
         yoQaDiag[idx] = new YoDouble("jointKF_QaDiag_" + jointName, registry);
         yoQaCapBindCount[idx] = new YoInteger("jointKF_QaCapBind_" + jointName + "_count", registry);
      }
   }

   /**
    * Re-reads the LIVE per-joint rotor inertia and alpha into the flat arrays the Schur process-noise update
    * consumes. Called at the top of every {@link #updateProcessNoiseFromMassMatrix()}; allocation-free (n
    * primitive reads and writes into arrays allocated once at construction).
    */
   private void refreshRotorInertiaAndSigmaTau()
   {
      for (int i = 0; i < state.n; i++)
      {
         rotorInertiaDiag[i] = yoRotorInertia[i].getValue();
         sigmaTauPerJoint[i] = sigmaTauIsFallback[i] ? sigmaTauFallback : yoAlpha[i].getValue() * tauMaxPerJoint[i];
      }
   }

   private void allocate()
   {
      int n = state.n;
      int dim = state.dim;
      xtmp.reshape(dim, 1);
      Ptmp.reshape(dim, dim);
      F.reshape(dim, dim);
      Q.reshape(dim, dim);

      // Schur-complement scratch + two Cholesky solvers, pre-warmed at their fixed sizes so the per-tick
      // setA()/solve()/invert() never allocate (chol decomposes in place and reuses its internal buffers once
      // warmed). Cholesky both exploits and enforces SPD-ness: a non-PD M_bb or Lambda (bad model state) fails
      // setA and the previous Q is kept instead of a garbage inverse entering the filter.
      if (massMatrixCalculator != null)
      {
         int nN = numberOfNuisanceDOF;
         MNN.reshape(nN, nN);
         MNf.reshape(nN, n);
         Mff.reshape(n, n);
         MNNInvMNf.reshape(nN, n);
         Lambda.reshape(n, n);
         LambdaInv.reshape(n, n);
         Ytau.reshape(n, n);
         Qa.reshape(n, n);

         nuisanceMassMatrixSolver = LinearSolverFactory_DDRM.chol(nN);
         nuisanceMassMatrixSolver.setA(CommonOps_DDRM.identity(nN));
         // Warm the nuisance solver's multi-column solve buffers at RHS width n (M_NN X = M_Nf is N x n), so the
         // per-tick solve reuses them. The MNf scratch is still zero here — this is only a buffer warm-up.
         nuisanceMassMatrixSolver.solve(MNf, MNNInvMNf);

         schurSolver = LinearSolverFactory_DDRM.chol(n);
         schurSolver.setA(CommonOps_DDRM.identity(n));
      }
   }

   private void buildConstantTransition()
   {
      CommonOps_DDRM.setIdentity(F);
      for (int i = 0; i < state.n; i++)
         F.set(i, state.n + i, dt); // q_{k+1} = q_k + dt * qd_k ; qd and bias are constant with noise
   }

   private void buildProcessNoise()
   {
      int n = state.n;
      Q.zero();
      double sigmaAccel = parameters.sigmaAccel.getValue();
      double sa2 = sigmaAccel * sigmaAccel;
      double dt2 = dt * dt;
      double dt3 = dt2 * dt;

      // Scalar-CWNA joint blocks first: this is both the no-model path and the fallback the mass-matrix
      // update leaves in place if its very first computation fails (e.g. model not yet in a valid pose).
      for (int i = 0; i < n; i++)
      {
         Q.set(i, i, sa2 * dt3 / 3.0); // CT white noise accel block, per joint
         Q.set(i, n + i, sa2 * dt2 / 2.0); // q - qd cross term
         Q.set(n + i, i, sa2 * dt2 / 2.0);
         Q.set(n + i, n + i, sa2 * dt);
      }
      for (var e : state.imuToOrdinal.entrySet()) // bias random walk, per-IMU process noise
      {
         e.getKey().getAngularVelocityBiasProcessNoiseCovariance(Rimu);
         // Do not let a non-finite covariance (e.g. an IMU still booting at construction) poison Q: a single
         // NaN here spreads to P on the first predict and never recovers. Skip this IMU's bias random-walk
         // instead, and name it so the offender is obvious in the log.
         if (JointLevelKFPreFilter.containsNonFinite(Rimu))
         {
            LogTools.error("Non-finite angular-velocity bias process-noise covariance from IMU " + e.getKey().getSensorName()
                           + " at construction; skipping its Q contribution (its bias random-walk is disabled) so Q stays finite.");
            continue;
         }
         int col = 2 * n + 3 * e.getValue();
         for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
               Q.set(col + i, col + j, Q.get(col + i, col + j) + dt * Rimu.get(i, j));
      }

      // Overwrites the joint (q, qd) blocks with the mass-matrix Qa if a model is available; no-op otherwise.
      updateProcessNoiseFromMassMatrix();
   }

   /** EKF time update in isolation: x <- F x, P <- F P F^T + Q(q). */
   void predict()
   {
      // Q is state-dependent on the mass-matrix path (Qa = sigma_tau^2 Lambda(q)^-2), so refresh it from the
      // model's current configuration before propagating the covariance. No-op on the scalar fallback path.
      updateProcessNoiseFromMassMatrix();
      CommonOps_DDRM.mult(F, state.x, xtmp);
      state.x.set(xtmp);
      CommonOps_DDRM.mult(F, state.P, Ptmp);
      CommonOps_DDRM.multTransB(Ptmp, F, state.P); // FPF^T term
      CommonOps_DDRM.addEquals(state.P, Q);
   }

   /** True when Q's joint blocks come from the Schur-complement Qa (a robot model was provided). */
   boolean isUsingMassMatrixProcessNoise()
   {
      return massMatrixCalculator != null;
   }

   /**
    * Rebuilds the joint-space Van Loan blocks of Q from the Schur-complement acceleration noise
    * Qa = sigma_tau^2 Lambda(q)^-2 (SPEC §3.2). For scalar Sigma_tau = sigma_tau^2 I and symmetric Lambda this
    * is sigma_tau^2 Lambda^-2. Called once from {@link #buildProcessNoise()} and then at the top of every
    * {@link #predict()}, because M(q) — read from the live model joints, whose frames the estimator refreshes
    * each tick — is configuration-dependent. Only the (q, qd) blocks are touched; the bias random-walk block and
    * the zero joint/bias coupling are left exactly as {@link #buildProcessNoise()} made them.
    *
    * <p>Numerical failure (non-finite M, Cholesky rejection of a non-PD M_NN or Lambda, non-finite
    * intermediate) leaves the previous Q in place — which at worst is the scalar-CWNA build — and warns once.
    * Allocation-free: the calculator, all scratch matrices, and both Cholesky solvers are pre-sized/warmed in
    * {@link #allocate()}.</p>
    *
    * <p>TODO(retune): SPEC §8 — Lambda^-2 ⪰ M_jj^-2, so the effective Qa grows at fixed sigma_tau relative to
    * the Rev. 1 locked-base map. sigma_tau must be retuned against quiet-standing and walking NIS by a human;
    * do NOT carry the Rev. 1 value over as if it were still calibrated.</p>
    */
   void updateProcessNoiseFromMassMatrix()
   {
      if (massMatrixCalculator == null)
         return;

      int n = state.n;
      // Pull this tick's LIVE per-joint rotor inertia and alpha into the flat arrays used below, so an SCS edit
      // takes effect on the very next predict.
      refreshRotorInertiaAndSigmaTau();

      massMatrixCalculator.reset();
      // Full mass matrix over {6-DoF base} ∪ {filtered joints}, ordered by the calculator's index provider.
      DMatrixRMaj massMatrix = massMatrixCalculator.getMassMatrix();
      if (JointLevelKFPreFilter.containsNonFinite(massMatrix))
      {
         warnMassMatrixFailureOnce("non-finite mass matrix");
         return;
      }

      // Extract the blocks by resolved column index (never assume ordering — SPEC §3.2). M is symmetric, so
      // M_jN = M_Nj^T and we only need M_NN, M_Nf, M_ff. M_ff is read in filter joint state order (row i =
      // filtered joint i), so Lambda and Lambda^-2 come out already in state order — no permutation in the fill.
      int nN = numberOfNuisanceDOF;
      for (int a = 0; a < nN; a++)
      {
         int ca = massMatrixNuisanceColumns[a];
         for (int b = 0; b < nN; b++)
            MNN.set(a, b, massMatrix.get(ca, massMatrixNuisanceColumns[b])); // M_NN (NxN)
         for (int j = 0; j < n; j++)
            MNf.set(a, j, massMatrix.get(ca, massMatrixColumn[j]));          // M_Nf (Nxn) = M_jN^T
      }
      for (int i = 0; i < n; i++)
      {
         int ci = massMatrixColumn[i];
         for (int j = 0; j < n; j++)
            Mff.set(i, j, massMatrix.get(ci, massMatrixColumn[j]));          // M_ff (nxn)
      }

      // Gap-joint reflected rotor inertia on the nuisance-block diagonal (Part B item 1): 0 on the 6 base rows,
      // the joint's reflected rotor inertia on each unfiltered gap joint (none on Alex; kept general). Floors
      // M_NN the same way the filtered rotor diagonal floors Lambda_eff below.
      if (nuisanceRotorInertiaDiag != null)
         for (int a = 0; a < nN; a++)
            MNN.add(a, a, nuisanceRotorInertiaDiag[a]);

      // X = M_NN^-1 M_Nf via Cholesky (rejects a non-PD nuisance block before it can enter the Schur complement).
      if (!nuisanceMassMatrixSolver.setA(MNN))
      {
         warnMassMatrixFailureOnce("nuisance mass-matrix block M_NN not positive definite");
         return;
      }
      nuisanceMassMatrixSolver.solve(MNf, MNNInvMNf); // X (Nxn); solve leaves MNf unmodified
      if (JointLevelKFPreFilter.containsNonFinite(MNNInvMNf))
      {
         warnMassMatrixFailureOnce("non-finite M_NN^-1 M_Nf (near-singular nuisance block)");
         return;
      }

      // Lambda = M_ff - M_jN X = M_ff - M_Nf^T X (M symmetric => M_jN = M_Nf^T). SPEC §3.2.
      CommonOps_DDRM.multTransA(MNf, MNNInvMNf, Lambda); // Lambda <- M_Nf^T X
      CommonOps_DDRM.changeSign(Lambda);                 // Lambda <- -M_jN X
      CommonOps_DDRM.addEquals(Lambda, Mff);             // Lambda <- M_ff - M_jN X
      // Symmetrize before the Cholesky invert: the block extraction is exact but the matrix products leave
      // Lambda symmetric only to round-off, and Cholesky assumes exact symmetry.
      JointLevelKFPreFilter.symmetrize(Lambda);

      // Lambda_eff = Lambda + diag(reflected rotor inertia) (Part B item 1, the primary fix). The rotor spins
      // behind the gearbox about its own axis and does not couple through the floating base, so this post-Schur
      // diagonal add is the EXACT drivetrain term — and simultaneously a principled regularizer: by Weyl it
      // floors lambda_min(Lambda_eff) >= min_i rotorInertiaDiag[i], so Lambda_eff^-1 has no proximal-joint
      // outliers and Qa cannot blow up the way the un-floored sigma_tau^2 Lambda^-2 did on Alex002.
      for (int i = 0; i < n; i++)
         Lambda.add(i, i, rotorInertiaDiag[i]);

      if (!schurSolver.setA(Lambda)) // Cholesky: rejects a non-PD Lambda_eff before it can enter Q
      {
         warnMassMatrixFailureOnce("Schur complement Lambda_eff not positive definite");
         return;
      }
      schurSolver.invert(LambdaInv); // LambdaInv = Lambda_eff^-1 (symmetric)
      if (JointLevelKFPreFilter.containsNonFinite(LambdaInv))
      {
         warnMassMatrixFailureOnce("non-finite Schur-complement inverse (near-singular Lambda_eff)");
         return;
      }

      // Per-joint Sigma_tau via the Gram form (Part B item 3): Y = Lambda_eff^-1 with column j scaled by
      // sigma_tau,j, then Qa = Y Y^T = Lambda_eff^-1 diag(sigma_tau^2) Lambda_eff^-T. Qa is PSD AND exactly
      // symmetric by construction, so the Van Loan fill below reads it directly (no symmetrized read). This
      // replaces the scalar Qa = sigma_tau^2 Lambda^-2; per-joint sigma_tau = alpha * tau_max,i scales the
      // process noise with each actuator's own torque capacity instead of one number across hip and wrist.
      for (int i = 0; i < n; i++)
         for (int j = 0; j < n; j++)
            Ytau.set(i, j, LambdaInv.get(i, j) * sigmaTauPerJoint[j]); // scale column j by sigma_tau,j
      CommonOps_DDRM.multTransB(Ytau, Ytau, Qa); // Qa = Y Y^T

      // QA_MAX tripwire (Part B item 2 — SURFACE, do not rescale). With the rotor floor on Lambda_eff, max
      // diag(Qa) must sit far below QA_MAX; if it ever would not, that is a model/config regression. Warn once
      // (naming the argmax joint by state index -> name) and count every tick it would bind — but keep Qa as
      // computed. The old uniform CommonOps_DDRM.scale coupled one joint's outlier into GLOBAL Q starvation
      // (hips down ~6 orders), which collapsed P onto the measurement floors and CAUSED the min-side S
      // singularity; a cap that silently rescales the whole robot is worse than the disease.
      double maxQaDiag = 0.0;
      int argMaxJoint = 0;
      for (int i = 0; i < n; i++)
      {
         double qaDiag = Qa.get(i, i);
         // Per-joint acceleration process-noise variance, the input the ALPHA equalization is calibrated from.
         yoQaDiag[i].set(qaDiag);
         if (qaDiag > maxQaDiag)
         {
            maxQaDiag = qaDiag;
            argMaxJoint = i;
         }
      }
      if (maxQaDiag > parameters.qaMax.getValue())
      {
         yoQaCapWouldBindCount.increment();
         yoQaCapBindCount[argMaxJoint].increment(); // per-joint attribution: WHICH joint's alpha needs knocking down
         warnQaCapWouldBindOnce(argMaxJoint, maxQaDiag);
      }

      double dt2 = dt * dt;
      double dt3 = dt2 * dt;
      for (int i = 0; i < n; i++)
      {
         for (int j = 0; j < n; j++)
         {
            // Qa is exactly symmetric by construction (Y Y^T), so read directly (Part B item 3 removed the
            // 0.5*(a_ij + a_ji) symmetrized read). Van Loan dt^3/3, dt^2/2, dt structure unchanged (SPEC §3.4).
            double qa = Qa.get(i, j);
            Q.set(i, j, qa * dt3 / 3.0);     // q-q block
            Q.set(i, n + j, qa * dt2 / 2.0); // q-qd cross term
            Q.set(n + i, j, qa * dt2 / 2.0);
            Q.set(n + i, n + j, qa * dt);    // qd-qd block
         }
      }
   }

   /** One-shot warning for mass-matrix Q failures; the filter keeps running on the previous (finite) Q. */
   private void warnMassMatrixFailureOnce(String reason)
   {
      if (warnedMassMatrixFailure)
         return;
      warnedMassMatrixFailure = true;
      LogTools.warn("Mass-matrix process-noise update failed (" + reason + "); keeping the previous Q"
            + " (scalar CWNA if this is the first computation). Reported once.");
   }

   /** One-shot warning that the Qa TRIPWIRE (QA_MAX) would bind (Part B item 2): max diag(Qa) exceeded the
    *  physical acceleration-noise ceiling. NOT rescaled — surfaced. With the reflected-rotor-inertia floor on
    *  Lambda_eff this should never fire; if it does, the named joint is a model/config regression (a rotor-table
    *  gap, a bad mass matrix, or a genuinely near-singular Lambda_eff) to chase, and jointKFQaCapWouldBindCount
    *  counts every tick it binds. */
   private void warnQaCapWouldBindOnce(int argMaxJointStateIndex, double maxQaDiag)
   {
      if (warnedMassMatrixConditioningCap)
         return;
      warnedMassMatrixConditioningCap = true;
      LogTools.warn("Joint-level KF process noise: max diag(Qa) = " + maxQaDiag + " (rad/s^2)^2 exceeds the QA_MAX tripwire ("
            + parameters.qaMax.getValue() + ") at joint '" + state.jointNameByStateIndex(argMaxJointStateIndex) + "'. Qa is NOT rescaled (a uniform "
            + "scale would starve global Q, the old failure mode); this indicates a model/config regression — check the "
            + "reflected-rotor-inertia table entry and the mass matrix for this joint. Counting in jointKFQaCapWouldBindCount. Reported once.");
   }

   /**
    * Finds the 6-DoF free-flyer joint at the root of the filtered joints' tree (SPEC §3.2 partitions the mass
    * matrix over a 6-DoF base). In an IHMC model this is the elevator's single child joint (the SixDoFJoint /
    * root joint). Returns {@code null} if no 6-DoF joint sits directly below the tree root (fixed-base model),
    * which drops the filter to the scalar-CWNA fallback. Construction-only; no hot-path use.
    */
   private static JointReadOnly findFloatingBaseJoint(RigidBodyBasics treeRoot)
   {
      for (JointReadOnly childJoint : treeRoot.getChildrenJoints())
      {
         if (childJoint.getDegreesOfFreedom() == 6)
            return childJoint;
      }
      return null;
   }

   /**
    * Collects every joint on the tree paths from {@code baseJoint} to each filtered joint: the filtered joints
    * themselves plus any unfiltered "gap" joints above them. Walks up from each filtered joint via
    * predecessor/parent-joint links until it reaches {@code baseJoint}. Order is insertion order (irrelevant —
    * all columns are later resolved by index). Construction-only; no hot-path use.
    */
   private static LinkedHashSet<JointReadOnly> collectSpanningJoints(JointReadOnly baseJoint, List<OneDoFJointBasics> filteredJoints)
   {
      LinkedHashSet<JointReadOnly> spanning = new LinkedHashSet<>();
      for (OneDoFJointBasics filteredJoint : filteredJoints)
      {
         JointReadOnly joint = filteredJoint;
         while (joint != null && joint != baseJoint)
         {
            spanning.add(joint);
            joint = joint.getPredecessor().getParentJoint(); // step one link toward the root
         }
      }
      return spanning;
   }
}

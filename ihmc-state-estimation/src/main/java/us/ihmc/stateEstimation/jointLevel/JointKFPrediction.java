package us.ihmc.stateEstimation.jointLevel;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

/**
 * The joint-level KF's time update: the constant transition F, the process noise Q, and the Schur-complement
 * mass-matrix path that makes Q configuration-dependent.
 *
 * <p>Process noise (SPEC §3.2): unmodeled joint torque maps to joint acceleration through the FLOATING-BASE
 * dynamics, not the locked-base map, so delta_qddot = lambda^-1 w_tau with
 * lambda = M_jj - M_jN M_NN^-1 M_Nj the Schur complement of the TORQUE-FREE block, and Qa = sigma_tau^2 lambda^-2.</p>
 *
 * <p>Every considered DoF falls into one of three groups, and which group a DoF is in IS the modeling choice
 * this class makes:</p>
 * <ul>
 *   <li><b>filtered</b> (f) — the filter's own joints; these carry the unmodeled torque w_tau.</li>
 *   <li><b>torque-free</b> (N) — the 6-DoF floating base plus any UNFILTERED joints on the tree path between it
 *       and a filtered joint ("gap" joints). Free to RECOIL under w_tau, which is what Schur-eliminating them
 *       expresses. Note "torque-free" means these rows carry no UNMODELED-TORQUE input, not that they are
 *       unactuated: the base obviously takes gravity and contact wrenches, and a gap joint has a real actuator.
 *       It is exact within this model because Qa maps w_tau only.</li>
 *   <li><b>locked</b> — genuinely off-path joints (e.g. arms when only legs are filtered). Stay ignored, so the
 *       composite-rigid-body calculator welds them into the adjacent link's inertia.</li>
 * </ul>
 *
 * <p>N is therefore the free/welded boundary of the disturbance-response model. On the real robot there are no
 * gap joints, so N is exactly the SPEC's 6-DoF base and lambda is exactly its M_jj - M_jb M_bb^-1 M_bj. Gap
 * joints are a topology generalization, marginalized rather than locked because the composite-rigid-body
 * calculator prunes the whole subtree below any ignored joint (locking one would zero the filtered joints'
 * inertia). Marginalizing is also the conservative choice: lambda &preceq; M_ff, so more recoil => more process
 * noise.</p>
 *
 * <p>All columns are resolved through the calculator's index provider — never assume ordering. A null
 * calculator (no robot model, or no floating base) falls back to the scalar-CWNA diagonal.</p>
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
   private int numberOfTorqueFreeDoFs;        // 6 (base) + number of gap joints; the marginalized block width N
   private int numberOfGapJoints;             // unfiltered joints on a base->filtered path; 0 on Alex's topology
   private int[] massMatrixTorqueFreeColumns; // torque-free DoF columns of the full mass matrix (length N)
   private int[] massMatrixFilteredColumns;   // filter joint state index -> DoF column in the calculator's mass matrix
   private final DMatrixRMaj torqueFreeInertia = new DMatrixRMaj(0, 0);          // M_NN, N x N
   private final DMatrixRMaj torqueFreeFilteredCoupling = new DMatrixRMaj(0, 0); // M_Nf, N x n (= M_jN^T)
   private final DMatrixRMaj filteredJointInertia = new DMatrixRMaj(0, 0);       // M_ff, n x n
   /** X = M_NN^-1 M_Nf (N x n): the recoil of the torque-free DoFs per unit filtered-joint acceleration,
    *  qddot_N = -X qddot_f. Subtracting M_jN X from M_ff is exactly what makes lambda a free-base inertia. */
   private final DMatrixRMaj recoilMap = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj lambda = new DMatrixRMaj(0, 0);    // Schur complement lambda_eff (rotor diag added in place), n x n
   private final DMatrixRMaj lambdaInv = new DMatrixRMaj(0, 0); // lambda_eff^-1, n x n
   private final DMatrixRMaj Ytau = new DMatrixRMaj(0, 0);      // lambda_eff^-1 with column j scaled by sigma_tau,j, n x n
   private final DMatrixRMaj Qa = new DMatrixRMaj(0, 0);        // Qa = Ytau Ytau^T (PSD, symmetric BY CONSTRUCTION), n x n
   private LinearSolverDense<DMatrixRMaj> torqueFreeMassMatrixSolver; // Cholesky over M_NN, solves M_NN X = M_Nf
   private LinearSolverDense<DMatrixRMaj> schurSolver;               // Cholesky over lambda_eff, inverts it

   // Filter-order arrays refreshed every predict from the LIVE YoVariables below; the torque-free one is fixed
   // at construction (a gap joint has no tunable of its own — none exist on Alex).
   private double[] rotorInertiaDiag;           // length n, added to lambda's diagonal (filter joint state order)
   private double[] torqueFreeRotorInertiaDiag; // length N: 0 on base rows, rotor inertia on gap-joint rows
   private double[] sigmaTauPerJoint;         // length n, sigma_tau,i = alpha_i * tau_max,i
   private YoDouble[] yoAlpha;
   private YoDouble[] yoRotorInertia;
   private double[] tauMaxPerJoint;      // length n, effort limit (N*m); NaN where the fallback applies
   private boolean[] sigmaTauIsFallback; // length n, true => no finite effort limit, use sigmaTauFallback
   private double sigmaTauFallback;      // boot-time read of jointKFParam_sigmaTau

   private boolean warnedMassMatrixFailure = false;
   private boolean warnedMassMatrixConditioningCap = false;

   private final YoInteger yoQaCapWouldBindCount;
   // Per-joint process-noise diagnostics, indexed by joint state index. yoQaDiag[i] is this tick's diag(Qa)_i
   // ((rad/s^2)^2) — the quantity QA_MAX thresholds, and the measurement the ALPHA equalization calibrates
   // from. The aggregate counter says the cap binds but not WHICH joint, so it cannot drive that retune;
   // yoQaCapBindCount closes it by counting the ticks joint i was the offending argmax.
   private YoDouble[] yoQaDiag;
   private YoInteger[] yoQaCapBindCount;

   JointKFPrediction(JointKFState state, RigidBodyBasics rootBody, JointKFParameters parameters, YoRegistry registry)
   {
      this.state = state;
      this.parameters = parameters;
      this.dt = state.dt;

      yoQaCapWouldBindCount = new YoInteger("jointKFQaCapWouldBindCount", registry);

      setupMassMatrix(rootBody, registry);
      // ORDER IS LOAD-BEARING: these two fill the rotor/sigma_tau and Qa telemetry arrays that the first
      // buildProcessNoise below reads. Creating the Qa variables here (rather than after, behind null guards)
      // publishes the boot-time Qa like every later tick's.
      createRotorInertiaAndSigmaTauParameters(registry);
      createQaYoVariables(registry);
      allocate();
      buildConstantTransition();
      buildProcessNoise();
   }

   private void setupMassMatrix(RigidBodyBasics rootBody, YoRegistry registry)
   {
      int n = state.numberOfJoints;
      // Built over the LIVE estimator model's joints (the same objects the Jacobians read), so M(q) tracks the
      // consumer-updated configuration with no extra bookkeeping. The floating joint MUST be included, or M_bb
      // and M_bj are unavailable for the Schur complement.
      if (rootBody != null && n > 0)
      {
         List<OneDoFJointBasics> massMatrixJoints = List.of(state.jointsByIndex); // filter state order, by definition
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
            // Consider = base + everything spanning base->filtered. Gap joints MUST be considered, not ignored:
            // the calculator prunes the whole subtree below an ignored joint, which would zero the filtered
            // joints' inertia if an unfiltered joint sat above them.
            LinkedHashSet<JointReadOnly> spanningJoints = collectSpanningJoints(baseJoint, massMatrixJoints);
            List<JointReadOnly> jointsToConsider = new ArrayList<>();
            jointsToConsider.add(baseJoint);
            jointsToConsider.addAll(spanningJoints);
            MultiBodySystemReadOnly massMatrixInput = MultiBodySystemReadOnly.toMultiBodySystemInput(jointsToConsider);
            massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(massMatrixInput);

            var indexProvider = massMatrixInput.getJointMatrixIndexProvider();
            // Filtered-joint columns, in filter state order.
            massMatrixFilteredColumns = new int[n];
            for (int i = 0; i < n; i++)
               massMatrixFilteredColumns[i] = indexProvider.getJointDoFIndices(state.jointsByIndex[i])[0];
            // Torque-free = base 6-DoF columns + each gap joint's column, i.e. every considered DoF that is not
            // a filtered joint. Its parallel rotor diagonal is 0 on the base rows (a free base carries no
            // reflected rotor inertia) so a marginalized gap joint still floors M_NN.
            int[] baseColumns = indexProvider.getJointDoFIndices(baseJoint);
            int torqueFreeCapacity = baseColumns.length + spanningJoints.size();
            TIntArrayList torqueFreeColumns = new TIntArrayList(torqueFreeCapacity);
            TDoubleArrayList torqueFreeRotorInertia = new TDoubleArrayList(torqueFreeCapacity);
            for (int c : baseColumns)
            {
               torqueFreeColumns.add(c);
               torqueFreeRotorInertia.add(0.0);
            }
            numberOfGapJoints = 0;
            for (JointReadOnly spanningJoint : spanningJoints)
            {
               if (!state.isFilteredJoint(spanningJoint)) // unfiltered => gap joint => marginalize it
               {
                  torqueFreeColumns.add(indexProvider.getJointDoFIndices(spanningJoint)[0]);
                  // Gap joints take the table value or the conservative default, silently — unlike a filtered
                  // joint, whose unmatched case is worth a warn (see createRotorInertiaAndSigmaTauParameters).
                  torqueFreeRotorInertia.add(JointKFParameters.reflectedRotorInertiaForNameOrDefault(spanningJoint.getName()));
                  numberOfGapJoints++;
               }
            }
            massMatrixTorqueFreeColumns = torqueFreeColumns.toArray(); // exact-size copies; no boxed drain loop
            torqueFreeRotorInertiaDiag = torqueFreeRotorInertia.toArray();
            numberOfTorqueFreeDoFs = massMatrixTorqueFreeColumns.length; // 6 + gap joints

            LogTools.info("Joint-level KF process noise: Schur-complement path (sigma_tau = " + parameters.sigmaTau.getValue() + " N*m) over "
                          + n + " joints, marginalizing a " + numberOfTorqueFreeDoFs + "-DoF torque-free block (6-DoF floating base"
                          + (numberOfGapJoints > 0 ? " + " + numberOfGapJoints + " unfiltered gap joint(s))." : ")."));
         }
      }
      else
      {
         LogTools.info("Joint-level KF process noise: scalar CWNA fallback (no robot model provided).");
      }
   }

   /**
    * Publishes the per-joint rotor inertia and alpha as LIVE YoVariables in filter state order, seeded from the
    * {@link JointKFParameters} tables, so the ALPHA equalization can be iterated in SCS against
    * jointKF_QaDiag_&lt;joint&gt;. A joint with no table match gets the default floor (warn); a joint with no
    * finite effort limit is pinned to the sigmaTau fallback (warn) and its alpha is then ignored.
    */
   private void createRotorInertiaAndSigmaTauParameters(YoRegistry registry)
   {
      int n = state.numberOfJoints;
      rotorInertiaDiag = new double[n];
      sigmaTauPerJoint = new double[n];
      yoRotorInertia = new YoDouble[n];
      yoAlpha = new YoDouble[n];
      tauMaxPerJoint = new double[n];
      sigmaTauIsFallback = new boolean[n];
      sigmaTauFallback = parameters.sigmaTau.getValue();

      for (int stateIndex = 0; stateIndex < n; stateIndex++)
      {
         OneDoFJointBasics joint = state.jointsByIndex[stateIndex];
         String jointName = joint.getName();

         if (JointKFParameters.isRotorInertiaUnmatched(jointName))
            LogTools.warn("JointLevelKFPreFilter: no reflected-rotor-inertia table entry for filtered joint '"
                          + jointName + "'; applying the conservative default floor "
                          + parameters.rotorInertiaDefault.getValue() + " kg*m^2.");
         yoRotorInertia[stateIndex] = new YoDouble("jointKFParam_rotorInertia_" + jointName,
                                            "LIVE: reflected rotor inertia n^2 J_rotor (kg*m^2) added to lambda's diagonal "
                                            + "before inversion. Floors lambda_min(lambda_eff), so lowering it inflates Qa.",
                                            registry);
         yoRotorInertia[stateIndex].set(JointKFParameters.reflectedRotorInertiaForNameOrDefault(jointName));

         double tauMax = joint.getEffortLimitUpper();
         sigmaTauIsFallback[stateIndex] = !Double.isFinite(tauMax) || tauMax <= 0.0;
         if (sigmaTauIsFallback[stateIndex])
         {
            LogTools.warn("JointLevelKFPreFilter: filtered joint '" + jointName + "' has no finite positive effort"
                          + " limit (" + tauMax + "); falling back to the scalar SIGMA_TAU = " + sigmaTauFallback
                          + " N*m for its sigma_tau.");
            tauMaxPerJoint[stateIndex] = Double.NaN; // unused on the fallback path; NaN so a regression is loud
         }
         else
         {
            tauMaxPerJoint[stateIndex] = tauMax;
         }
         yoAlpha[stateIndex] = new YoDouble("jointKFParam_alpha_" + jointName,
                                     "LIVE: fraction of this joint's torque capacity treated as unmodeled; "
                                     + "sigma_tau = alpha * tau_max. Retune against jointKF_QaDiag_" + jointName
                                     + " to equalize sqrt(diag(Qa)) at jointKFParam_targetQddStd."
                                     + (sigmaTauIsFallback[stateIndex] ? " IGNORED for this joint: no finite effort limit, so "
                                                                  + "jointKFParam_sigmaTau is used instead." : ""),
                                     registry);
         yoAlpha[stateIndex].set(JointKFParameters.alphaForName(jointName));
      }

      refreshRotorInertiaAndSigmaTau();
   }

   private void createQaYoVariables(YoRegistry registry)
   {
      int n = state.numberOfJoints;
      yoQaDiag = new YoDouble[n];
      yoQaCapBindCount = new YoInteger[n];
      for (int stateIndex = 0; stateIndex < n; stateIndex++)
      {
         String jointName = state.jointsByIndex[stateIndex].getName();
         yoQaDiag[stateIndex] = new YoDouble("jointKF_QaDiag_" + jointName, registry);
         yoQaCapBindCount[stateIndex] = new YoInteger("jointKF_QaCapBind_" + jointName + "_count", registry);
      }
   }

   /**
    * Re-reads the LIVE per-joint rotor inertia and alpha into the flat arrays the Schur update consumes.
    * Allocation-free: n primitive reads into arrays allocated once at construction.
    */
   private void refreshRotorInertiaAndSigmaTau()
   {
      for (int i = 0; i < state.numberOfJoints; i++)
      {
         rotorInertiaDiag[i] = yoRotorInertia[i].getValue();
         sigmaTauPerJoint[i] = sigmaTauIsFallback[i] ? sigmaTauFallback : yoAlpha[i].getValue() * tauMaxPerJoint[i];
      }
   }

   private void allocate()
   {
      int n = state.numberOfJoints;
      int dim = state.dim;
      xtmp.reshape(dim, 1);
      Ptmp.reshape(dim, dim);
      F.reshape(dim, dim);
      Q.reshape(dim, dim);

      // Scratch + both Cholesky solvers pre-warmed at their fixed sizes, so the per-tick setA/solve/invert never
      // allocate. Cholesky also ENFORCES SPD-ness: a non-PD M_NN or lambda fails setA and the previous Q is kept
      // rather than a garbage inverse entering the filter.
      if (massMatrixCalculator != null)
      {
         torqueFreeInertia.reshape(numberOfTorqueFreeDoFs, numberOfTorqueFreeDoFs);
         torqueFreeFilteredCoupling.reshape(numberOfTorqueFreeDoFs, n);
         filteredJointInertia.reshape(n, n);
         recoilMap.reshape(numberOfTorqueFreeDoFs, n);
         lambda.reshape(n, n);
         lambdaInv.reshape(n, n);
         Ytau.reshape(n, n);
         Qa.reshape(n, n);

         torqueFreeMassMatrixSolver = LinearSolverFactory_DDRM.chol(numberOfTorqueFreeDoFs);
         torqueFreeMassMatrixSolver.setA(CommonOps_DDRM.identity(numberOfTorqueFreeDoFs));
         // Warm the torque-free solver's multi-column solve buffers at RHS width n (M_NN X = M_Nf is N x n), so the
         // per-tick solve reuses them. The torqueFreeFilteredCoupling scratch is still zero here — this is only a buffer warm-up.
         torqueFreeMassMatrixSolver.solve(torqueFreeFilteredCoupling, recoilMap);

         schurSolver = LinearSolverFactory_DDRM.chol(n);
         schurSolver.setA(CommonOps_DDRM.identity(n));
      }
   }

   private void buildConstantTransition()
   {
      CommonOps_DDRM.setIdentity(F);
      for (int i = 0; i < state.numberOfJoints; i++)
         F.set(i, state.numberOfJoints + i, dt); // q_{k+1} = q_k + dt * qd_k ; qd and bias are constant with noise
   }

   private void buildProcessNoise()
   {
      int n = state.numberOfJoints;
      Q.zero();
      double sigmaAccel = parameters.sigmaAccel.getValue();
      double sigmaAccelSquared = sigmaAccel * sigmaAccel;
      double dt2 = dt * dt;
      double dt3 = dt2 * dt;

      // Scalar-CWNA joint blocks first: this is both the no-model path and the fallback the mass-matrix
      // update leaves in place if its very first computation fails (e.g. model not yet in a valid pose).
      for (int i = 0; i < n; i++)
      {
         Q.set(i, i, sigmaAccelSquared * dt3 / 3.0); // CT white noise accel block, per joint
         Q.set(i, n + i, sigmaAccelSquared * dt2 / 2.0); // q - qd cross term
         Q.set(n + i, i, sigmaAccelSquared * dt2 / 2.0);
         Q.set(n + i, n + i, sigmaAccelSquared * dt);
      }
      for (int o = 0; o < state.numberOfIMUs; o++) // bias random walk, per-IMU process noise
      {
         IMUSensorReadOnly imu = state.imusByOrdinal[o];
         imu.getAngularVelocityBiasProcessNoiseCovariance(Rimu);
         // Do not let a non-finite covariance (e.g. an IMU still booting at construction) poison Q: a single
         // NaN here spreads to P on the first predict and never recovers. Skip this IMU's bias random-walk
         // instead, and name it so the offender is obvious in the log.
         if (JointLevelKFPreFilter.containsNonFinite(Rimu))
         {
            LogTools.error("Non-finite angular-velocity bias process-noise covariance from IMU " + imu.getSensorName()
                           + " at construction; skipping its Q contribution (its bias random-walk is disabled) so Q stays finite.");
            continue;
         }
         int col = 2 * n + 3 * o;
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
      // Q is state-dependent on the mass-matrix path (Qa = sigma_tau^2 lambda(q)^-2), so refresh it from the
      // model's current configuration before propagating the covariance. No-op on the scalar fallback path.
      updateProcessNoiseFromMassMatrix();
      CommonOps_DDRM.mult(F, state.x, xtmp);
      state.x.set(xtmp);
      CommonOps_DDRM.mult(F, state.P, Ptmp);
      CommonOps_DDRM.multTransB(Ptmp, F, state.P); // FPF^T term
      CommonOps_DDRM.addEquals(state.P, Q);
      // FPF^T + Q is symmetric in exact arithmetic; force it so round-off can't drift P before the encoder/pair
      // updates read it this tick (matches the JAX reference's LSigmaL^T discipline).
      JointLevelKFPreFilter.symmetrize(state.P);
   }

   /** True when Q's joint blocks come from the Schur-complement Qa (a robot model was provided). */
   boolean isUsingMassMatrixProcessNoise()
   {
      return massMatrixCalculator != null;
   }

   /**
    * Rebuilds Q's joint-space Van Loan blocks from Qa = sigma_tau^2 lambda(q)^-2. Runs at the top of every
    * {@link #predict()} because M(q) is configuration-dependent. Only the (q, qd) blocks are touched; the bias
    * random-walk block is left as {@link #buildProcessNoise()} made it. Any numerical failure leaves the
    * previous Q in place (at worst the scalar-CWNA build) and warns once. Allocation-free.
    *
    * <p>TODO(retune): lambda^-2 ⪰ M_jj^-2, so Qa grows at fixed sigma_tau relative to the Rev. 1 locked-base
    * map. sigma_tau must be retuned against quiet-standing and walking NIS by a human — do NOT carry the Rev. 1
    * value over as if it were still calibrated.</p>
    */
   void updateProcessNoiseFromMassMatrix()
   {
      if (massMatrixCalculator == null)
         return;

      int n = state.numberOfJoints;
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

      // Extract by resolved column index, never by assumed ordering. M is symmetric, so M_jN = M_Nf^T and only
      // M_NN, M_Nf, M_ff are needed. M_ff is read in filter state order, so lambda comes out already in state
      // order — no permutation in the fill below.
      for (int a = 0; a < numberOfTorqueFreeDoFs; a++)
      {
         int torqueFreeColumn = massMatrixTorqueFreeColumns[a];
         for (int b = 0; b < numberOfTorqueFreeDoFs; b++)
            torqueFreeInertia.set(a, b, massMatrix.get(torqueFreeColumn, massMatrixTorqueFreeColumns[b])); // M_NN (NxN)
         for (int j = 0; j < n; j++)
            torqueFreeFilteredCoupling.set(a, j, massMatrix.get(torqueFreeColumn, massMatrixFilteredColumns[j]));          // M_Nf (Nxn) = M_jN^T
      }
      for (int i = 0; i < n; i++)
      {
         int filteredColumn = massMatrixFilteredColumns[i];
         for (int j = 0; j < n; j++)
            //TODO: use this via block operations?
            filteredJointInertia.set(i, j, massMatrix.get(filteredColumn, massMatrixFilteredColumns[j]));          // M_ff (nxn)
      }

      // Floors M_NN the same way the filtered rotor diagonal floors lambda_eff below.
      if (torqueFreeRotorInertiaDiag != null)
         for (int a = 0; a < numberOfTorqueFreeDoFs; a++)
            torqueFreeInertia.add(a, a, torqueFreeRotorInertiaDiag[a]);

      // X = M_NN^-1 M_Nf via Cholesky (rejects a non-PD torque-free block before it can enter the Schur complement).
      if (!torqueFreeMassMatrixSolver.setA(torqueFreeInertia))
      {
         warnMassMatrixFailureOnce("torque-free mass-matrix block M_NN not positive definite");
         return;
      }
      torqueFreeMassMatrixSolver.solve(torqueFreeFilteredCoupling, recoilMap); // X (Nxn); solve leaves torqueFreeFilteredCoupling unmodified
      if (JointLevelKFPreFilter.containsNonFinite(recoilMap))
      {
         warnMassMatrixFailureOnce("non-finite M_NN^-1 M_Nf (near-singular torque-free block)");
         return;
      }

      // lambda = M_ff - M_jN X = M_ff - M_Nf^T X (M symmetric => M_jN = M_Nf^T). SPEC §3.2.
      lambda.set(filteredJointInertia);
      CommonOps_DDRM.multAddTransA(-1.0, torqueFreeFilteredCoupling, recoilMap, lambda);
      // Symmetrize before the Cholesky invert: the block extraction is exact but the matrix products leave
      // lambda symmetric only to round-off, and Cholesky assumes exact symmetry.
      JointLevelKFPreFilter.symmetrize(lambda);
      //TODO: check if this is done internally by the EJML solver

      // lambda_eff = lambda + diag(rotor inertia). The rotor does not couple through the floating base, so this
      // post-Schur diagonal add is the EXACT drivetrain term and simultaneously a principled regularizer: by
      // Weyl it floors lambda_min(lambda_eff), which is what stops Qa blowing up as it did on Alex002.
      for (int i = 0; i < n; i++)
//         MatrixTools.addDiagonal(lambda, rotorInertiaDiag[i]);
         //TODO: tests fail when addDiagonal is used - need to figure out why.
         lambda.add(i,i, rotorInertiaDiag[i]);

      if (!schurSolver.setA(lambda)) // Cholesky: rejects a non-PD lambda_eff before it can enter Q
      {
         warnMassMatrixFailureOnce("Schur complement lambda_eff not positive definite");
         return;
      }
      schurSolver.invert(lambdaInv); // lambdaInv = lambda_eff^-1 (symmetric)
      if (JointLevelKFPreFilter.containsNonFinite(lambdaInv))
      {
         warnMassMatrixFailureOnce("non-finite Schur-complement inverse (near-singular lambda_eff)");
         return;
      }

      // Gram form: Y = lambda_eff^-1 with column j scaled by sigma_tau,j, so Qa = Y Y^T is PSD AND exactly
      // symmetric by construction and the Van Loan fill below can read it directly. Per-joint sigma_tau scales
      // the process noise with each actuator's own capacity instead of one number across hip and wrist.
      Ytau.set(lambdaInv);
      for (int col = 0; col < n; col++)
         CommonOps_DDRM.scaleCol(sigmaTauPerJoint[col], Ytau, col);
      CommonOps_DDRM.multOuter(Ytau, Qa);

      // QA_MAX tripwire: SURFACE, do not rescale. With the rotor floor on lambda_eff, max diag(Qa) must sit far
      // below it, so exceeding it is a model/config regression to chase. The old uniform rescale coupled one
      // joint's outlier into GLOBAL Q starvation (hips down ~6 orders), which collapsed P onto the measurement
      // floors and CAUSED the min-side S singularity — a cap that rescales the whole robot is worse than the disease.
      double maxQaDiag = 0.0;
      int argMaxJoint = 0;
      for (int i = 0; i < n; i++)
      {
         double qaDiag = Qa.get(i, i);
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
            double qaEntry = Qa.get(i, j); // exactly symmetric (Y Yᵀ), so no symmetrized read is needed
            Q.set(i, j, qaEntry * dt3 / 3.0);     // q-q block
            Q.set(i, n + j, qaEntry * dt2 / 2.0); // q-qd cross term
            Q.set(n + i, j, qaEntry * dt2 / 2.0);
            Q.set(n + i, n + j, qaEntry * dt);    // qd-qd block
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

   /** One-shot: max diag(Qa) exceeded the ceiling. With the rotor floor on lambda_eff this should never fire;
    *  if it does, the named joint is a regression to chase (a rotor-table gap, a bad mass matrix, or a genuinely
    *  near-singular lambda_eff). Surfaced, never rescaled. */
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
    * The 6-DoF free-flyer at the root of the filtered joints' tree — in an IHMC model, the elevator's single
    * child joint. Null on a fixed-base model, which drops the filter to the scalar-CWNA fallback.
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
    * Every joint on the tree paths from {@code baseJoint} to each filtered joint — the filtered joints plus any
    * unfiltered "gap" joints above them — by walking up predecessor links. Order is irrelevant; all columns are
    * later resolved by index.
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

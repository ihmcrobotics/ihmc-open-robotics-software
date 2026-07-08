package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUBasedJointStateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Joint-level Kalman filter pre-filter (P-A architecture): one filter over the IMU tree whose pair
 * measurements z_w,ab = J(q^) S_ab qd + b_w,ab + v couple the joints on each IMU-pair path, with
 * per-IMU biases in state and a phase-2 gauge anchor for the absolute base bias. When a robot model is
 * provided, the joint process noise is the mass-matrix-induced Qa = sigma_tau^2 M(q)^-2 of the write-up
 * (eqs. (10)-(12)), recomputed each predict; otherwise a scalar-CWNA diagonal fallback.
 *
 * <p><b>Do not share an instance between pipelines</b> (the filter holds its covariance).</p>
 *
 * <p><b>Rev. 2 process noise (SPEC §3.2).</b> The joint process noise is now the floating-base Schur
 * complement Qa = sigma_tau^2 * Lambda(q)^-2, Lambda = M_jj - M_jb M_bb^-1 M_bj, rather than the Rev. 1
 * locked-base Qa = sigma_tau^2 * M_jj^-2. Because the free base recoils, the joints accelerate more per unit
 * torque, so Lambda ⪯ M_jj (PSD ordering) and Lambda^-2 ⪰ M_jj^-2: the effective process noise grows at fixed
 * sigma_tau, worst for proximal joints. See the SPEC (jointKF_derivation.md §3.2, §8) for the derivation and
 * the sigma_tau retune obligation.</p>
 *
 * @author Lucas Libshutz
 */
public class JointLevelKFPreFilter implements ProprioceptivePreFilter
{
   //TUNING VARIABLES
   private static final double ENCODER_VAR = 1.0e-6; // (1-e-3 rad)^2 encoder position variance
   private static final double SIGMA_ACCEL = 50.0; // rad/s^2 CWNA process-noise STD (scalar fallback when no robot model is provided)
   private static final double SIGMA_TAU = 50.0; // N*m unmodeled-torque STD for the mass-matrix path: Qa = sigma_tau^2 M(q)^-2
   private static final double INIT_POS_VAR = 1.0e-6; // encoders trusted at initialization
   private static final double INIT_VEL_VAR = 1.0; // velocity unknown at initialization
   private static final double INIT_BIAS_VAR = 2.5e-3; // (0.05 rad/s)^2
   private static final double ANCHOR_VAR = 4.0e-4; // stance FK slip variance
   // On-ground initialization gate: the exported base-IMU gyro bias is only observable through the phase-2
   // stance anchor, which runs only when a foot is trusted. If the filter is seeded while the robot hangs
   // (feet off the ground) the base bias is unobservable, its covariance grows unbounded under the bias
   // random-walk, and the estimate wanders — which the downstream InEKF (no gyro-bias state of its own)
   // integrates straight into orientation. So we defer initialization until BOTH feet have been firmly in
   // contact for a short debounce window; while uninitialized the filter exports NaN joint states and zero
   // bias, so consumers cleanly fall back to the raw gyro/sensors. Debounced (not a single-tick check)
   // because the foot-switch contact-probability source seeds to 1.0 on the assumption feet are planted at
   // init, so a naive "both == 1" would false-pass on the first tick(s) precisely while hanging.
   private static final double ON_GROUND_INIT_DEBOUNCE = 0.05; // s of continuous ground contact before seeding

   // Declaration of all pre-allocated variables
   private final SensorOutputMapReadOnly sensorMap;
   private final double dt;

   private final LinkedHashMap<OneDoFJointBasics, Integer> jointToIndex = new LinkedHashMap<>();
   private final LinkedHashMap<IMUSensorReadOnly, Integer> imuToOrdinal = new LinkedHashMap<>();
   private final List<Pair> pairs = new ArrayList<>();
   private final List<FootAnchor> footAnchors = new ArrayList<>();
   private IMUSensorReadOnly baseIMU;

   private int baseBiasCol;
   private int n; // number of distinct joints
   private int m; // number of IMUs
   private int dim; // 2n + 3m
   private boolean initialized = false;
   private boolean warnedNonFiniteInput = false;

   // Optional on-ground gate (see ON_GROUND_INIT_DEBOUNCE). Null => no gating: the filter initializes as soon
   // as boot data is finite (the behavior the package tests rely on). Wired by the consumer that owns the
   // contact-probability signal (InvariantMainStateEstimator) via setInitializationGate.
   private BooleanSupplier onGroundGate = null;
   private int consecutiveOnGroundTicks = 0;
   private final int requiredOnGroundTicks;

   // Observability: the constructor was previously handed a parentRegistry that it dropped on the floor
   // (so the filter published nothing on hardware). These are wired to the parent registry now.
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoBoolean yoInitialized = new YoBoolean("jointKFInitialized", registry);
   private final YoBoolean yoWaitingForGroundContact = new YoBoolean("jointKFWaitingForGroundContact", registry);
   private final YoInteger yoStateDimension = new YoInteger("jointKFStateDimension", registry);
   private final YoInteger yoNumberOfFilteredJoints = new YoInteger("jointKFNumberOfFilteredJoints", registry);
   private final YoInteger yoNumberOfIMUs = new YoInteger("jointKFNumberOfIMUs", registry);

   // Per-joint filtered state (q, qd) and the 1-sigma covariance envelope around each. All indexed by the
   // joint's state index (the same index used into x and P). Allocated once in the constructor after n is
   // known, then only .set() on the estimator thread (allocation-free). The upper/lower "bounds" are the
   // estimate +/- one standard deviation, i.e. q +/- sqrt(P_qq) and qd +/- sqrt(P_qdqd), for plotting the
   // filter's confidence envelope alongside the estimate in SCS.
   private YoDouble[] yoJointPosition;
   private YoDouble[] yoJointVelocity;
   private YoDouble[] yoJointPositionUpperBound;
   private YoDouble[] yoJointPositionLowerBound;
   private YoDouble[] yoJointVelocityUpperBound;
   private YoDouble[] yoJointVelocityLowerBound;

   // State and constant model
   private final DMatrixRMaj x = new DMatrixRMaj(0,1); // reshaped to dimx1 later when constructed
   private final DMatrixRMaj xtmp = new DMatrixRMaj(0,1);
   private final DMatrixRMaj P = new DMatrixRMaj(0,0); // dim x dim
   private final DMatrixRMaj F = new DMatrixRMaj(0,0); // dim x dim transition matrix
   private final DMatrixRMaj Q = new DMatrixRMaj(0,0); // dim x dim process noise
   private DMatrixRMaj Henc, Renc, zEnc;

   // Measurement model matrices
   private final DMatrixRMaj H = new DMatrixRMaj(3,0);
   private final DMatrixRMaj PHt = new DMatrixRMaj(0,0);
   private final DMatrixRMaj S = new DMatrixRMaj(0,0);
   private final DMatrixRMaj Sinv = new DMatrixRMaj(0,0);
   private final DMatrixRMaj K = new DMatrixRMaj(0,0);
   private final DMatrixRMaj nu = new DMatrixRMaj(0,0);
   private final DMatrixRMaj KR = new DMatrixRMaj(0,0); // I-KH, dim x dim, Joseph form
   private final DMatrixRMaj IKH = new DMatrixRMaj(0,0); // I-KH, dim x dim, Joseph form
   private final DMatrixRMaj Ptmp = new DMatrixRMaj(0,0);
   private final DMatrixRMaj zMeas = new DMatrixRMaj(3,1);
   private final DMatrixRMaj zAnchor = new DMatrixRMaj(3,1);
   private final DMatrixRMaj R3 = new DMatrixRMaj(3,3);
   private final DMatrixRMaj Rimu = new DMatrixRMaj(3,3);
   private final DMatrixRMaj rot3 = new DMatrixRMaj(3,3);
   private final DMatrixRMaj tmp3a = new DMatrixRMaj(3,3);
   private final DMatrixRMaj tmp3b = new DMatrixRMaj(3,3);
   private final DMatrixRMaj anchorR = new DMatrixRMaj(3,3);
   // Reused LU solver for the innovation-covariance inverse, pre-sized in allocate(). This replaces the
   // per-tick CommonOps_DDRM.invert(S, Sinv), which allocates a fresh LU decomposition + solver on every
   // call once S is larger than 5x5 (i.e. the n-joint encoder update) — garbage on the estimator thread.
   private LinearSolverDense<DMatrixRMaj> innovationSolver;
   private final RigidBodyTransform tmpTransform = new RigidBodyTransform();
   private final FrameVector3D fvA = new FrameVector3D();
   private final FrameVector3D fvB = new FrameVector3D();
   private final FrameVector3D biasOut = new FrameVector3D();

   // Schur-complement process noise (SPEC §3.2). The unmodeled joint torque w_tau maps to joint acceleration
   // through the FLOATING-BASE dynamics, not the locked-base map: eliminating the (unforced) nuisance
   // acceleration from the perturbed EOM gives delta_qddot = Lambda^-1 w_tau, Lambda = M_jj - M_jN M_NN^-1 M_Nj
   // the Schur complement of the nuisance block. Hence Qa = sigma_tau^2 Lambda^-2.
   //
   // "Nuisance" = the 6-DoF floating base (SPEC §3.2) plus any UNFILTERED joints that lie on the tree path
   // between the base and a filtered joint ("gap" joints). In the real robot the base IMU is on the base link
   // and every joint on a pair/anchor path is itself filtered, so there are NO gap joints and this is exactly
   // the SPEC's Lambda = M_jj - M_jb M_bb^-1 M_bj over the plain 6-DoF base. The gap term is a topology
   // generalization (needed because the composite-rigid-body calculator prunes the subtree below any ignored
   // joint, so an unfiltered joint above a filtered one cannot be silently locked): such gap joints are
   // instead marginalized (treated as free) alongside the base — the conservative choice (more recoil => more
   // process noise, consistent with §3.2's free-flyer upper bound). Genuinely OFF-path joints stay ignored and
   // their inertia is composited into the adjacent link by the calculator (considerIgnoredSubtreesInertia).
   //
   // The mass matrix M is built over {base} ∪ {joints spanning base->filtered}. All columns (nuisance and each
   // filtered joint) are resolved through the calculator's index provider — never assume ordering. Null
   // calculator (no robot model, or no 6-DoF floating base found) => the constant scalar-CWNA diagonal fallback.
   private CompositeRigidBodyMassMatrixCalculator massMatrixCalculator;
   private int numNuisanceDoF;          // 6 (base) + number of gap joints; the marginalized block width N
   private int[] massMatrixNuisanceColumns; // nuisance DoF columns of the full mass matrix (length N)
   private int[] massMatrixColumn;      // filter joint state index -> DoF column in the calculator's mass matrix
   private final DMatrixRMaj MNN = new DMatrixRMaj(0, 0);       // nuisance block, N x N
   private final DMatrixRMaj MNf = new DMatrixRMaj(0, 0);       // nuisance-filtered coupling, N x n (= M_jN^T)
   private final DMatrixRMaj Mff = new DMatrixRMaj(0, 0);       // filtered block, n x n
   private final DMatrixRMaj MNNInvMNf = new DMatrixRMaj(0, 0); // X = M_NN^-1 M_Nf, N x n
   private final DMatrixRMaj Lambda = new DMatrixRMaj(0, 0);    // Schur complement, n x n
   private final DMatrixRMaj LambdaInv = new DMatrixRMaj(0, 0); // Lambda^-1, n x n
   private final DMatrixRMaj LambdaInvSq = new DMatrixRMaj(0, 0); // Lambda^-2, n x n (in filter joint state order)
   private LinearSolverDense<DMatrixRMaj> nuisanceMassMatrixSolver; // Cholesky over M_NN, solves M_NN X = M_Nf
   private LinearSolverDense<DMatrixRMaj> schurSolver;          // Cholesky over Lambda, inverts it
   private boolean warnedMassMatrixFailure = false;

   // Rollback backups: a KF has no recovery once x/P go non-finite (predict/Joseph spread the NaN with no
   // way back), so a single non-finite measurement update is skipped by restoring the pre-update mean and
   // covariance instead of latching NaN forever. Pre-sized in allocate(); .set() reuses them (no per-tick
   // allocation). The O(dim^2) copy is negligible next to the O(dim^3) Joseph matrix products already here.
   private final DMatrixRMaj xBackup = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj PBackup = new DMatrixRMaj(0, 0);
   // One-shot triage: names the first predict/update stage whose output goes non-finite, so a hardware run
   // prints exactly where the NaN enters. Latches after the first report (the KF stays NaN anyway).
   private boolean nonFiniteStateReported = false;


   // Package-private (not private) so the allocation/behavior tests in this package can build it directly
   // from a synthetic IMU-pair setup without standing up a full StateEstimatorParameters.
   /** Scalar-CWNA process-noise overload: no robot model, so Qa = SIGMA_ACCEL^2 I (the pre-mass-matrix behavior). */
   JointLevelKFPreFilter(SensorOutputMapReadOnly sensorMap,
                         List<IMUBasedJointStateEstimatorParameters> pairParameters,
                         Collection<RigidBodyBasics> feet,
                         double estimatorDT,
                         YoRegistry parentRegistry)
   {
      this(sensorMap, pairParameters, feet, null, estimatorDT, parentRegistry);
   }

   /**
    * @param rootBody root of the estimator's robot model (the elevator). When non-null, the joint process
    *                 noise is the mass-matrix-induced Qa = sigma_tau^2 M(q)^-2 (write-up eqs. (10)-(12)),
    *                 recomputed every predict because M depends on the configuration. When null, falls back
    *                 to the constant scalar-CWNA diagonal Qa = SIGMA_ACCEL^2 I.
    */
   JointLevelKFPreFilter(SensorOutputMapReadOnly sensorMap,
                         List<IMUBasedJointStateEstimatorParameters> pairParameters,
                         Collection<RigidBodyBasics> feet,
                         RigidBodyBasics rootBody,
                         double estimatorDT,
                         YoRegistry parentRegistry)
   {
      this.sensorMap = sensorMap;
      this.dt = estimatorDT;
      this.requiredOnGroundTicks = Math.max(1, (int) Math.round(ON_GROUND_INIT_DEBOUNCE / estimatorDT));
      if (parentRegistry != null)
         parentRegistry.addChild(registry);

      // 1)  Resolve all pairs of IMUs, collect distinct joints and IMUs into the state layout
      for (IMUBasedJointStateEstimatorParameters pp : pairParameters)
      {
         IMUSensorReadOnly parent = findIMU(sensorMap, pp.getParentIMUName());
         IMUSensorReadOnly child = findIMU(sensorMap, pp.getChildIMUName());
         if (parent == null || child == null)
         {
            LogTools.warn("Skipping pair, IMU not found: parent = " + pp.getParentIMUName() + " child = " + pp.getChildIMUName());
            continue;
         }
         Pair p = new Pair(parent,child);
         p.jac.setKinematicChain(parent.getMeasurementLink(),child.getMeasurementLink());
         p.jac.setJacobianFrame(child.getMeasurementLink().getBodyFixedFrame());
         //NOTE: Assumes 1-DoF and fixed joints only, just as IMUBasedJointStateEstimator does.
         // The i-th angular jacobian column correspnods to the i-th filtered joint.
         List<OneDoFJointBasics> chain = MultiBodySystemTools.filterJoints(p.jac.getJointsFromBaseToEndEffector(), OneDoFJointBasics.class);
         p.chainJoints = chain.toArray(new OneDoFJointBasics[0]);
         for (OneDoFJointBasics j : p.chainJoints)
            jointToIndex.putIfAbsent(j, jointToIndex.size()); // places joints via hash mapping
         imuToOrdinal.putIfAbsent(parent, imuToOrdinal.size());
         imuToOrdinal.putIfAbsent(child, imuToOrdinal.size());
         pairs.add(p);
      }

      n = jointToIndex.size();
      m = imuToOrdinal.size();
      dim = 2 * n + 3 * m;

      // 2) Second pass: assemble state sinze columns are known as n is fixed.
      for (Pair p: pairs)
      {
         int dof = p.chainJoints.length;
         p.Jang.reshape(3,dof); // Jang is allocated blank at construction in the chain, only reshaped once full DoF are known
         p.qdCols = new int[dof];
         for (int c = 0; c < dof; c++)
            p.qdCols[c] = n + jointToIndex.get(p.chainJoints[c]); // this int[] is the selector matrix of the path S_ab, but in sparse form as all joints are not related directly
         p.parentBias = 2 * n + 3 * imuToOrdinal.get(p.parent);
         p.childBias = 2 * n + 3 * imuToOrdinal.get(p.child);
      }

      // 3) Base IMU = root of the tree + first pair parent.
      //WARNING: Need to configure if the root differs.
      baseIMU = pairs.isEmpty() ? null : pairs.get(0).parent;
      baseBiasCol = baseIMU == null ? -1 : 2 * n + 3 * imuToOrdinal.get(baseIMU);
      if (baseIMU == null)
         throw new RuntimeException("Base IMU is null, check the kinematic tree.");
      LogTools.info("Base IMU initialized as " + baseIMU.getSensorName());

      // 3b) Schur-complement process noise (SPEC §3.2): the calculator is built over the FLOATING BASE plus the
      // filtered joints of the LIVE estimator model (the same joint objects the Jacobians read), so its M(q)
      // tracks the consumer-updated configuration each tick with no extra bookkeeping. Unlike Rev. 1 (joints
      // only), the floating joint MUST be included so the 6x6 base block M_bb and the base-joint coupling M_bj
      // are available for the Schur complement. Column mappings (base and each joint) are resolved through the
      // index provider rather than assumed from list order.
      if (rootBody != null && n > 0)
      {
         List<OneDoFJointBasics> massMatrixJoints = new ArrayList<>(jointToIndex.keySet());
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
            for (var e : jointToIndex.entrySet())
               massMatrixColumn[e.getValue()] = indexProvider.getJointDoFIndices(e.getKey())[0];
            // Nuisance columns to marginalize = base 6-DoF columns + each gap joint's column (a gap joint is a
            // spanning joint that is not itself filtered). These are exactly the considered DoFs that are not
            // filtered joints.
            int[] baseColumns = indexProvider.getJointDoFIndices(baseJoint);
            List<Integer> nuisanceColumns = new ArrayList<>();
            for (int c : baseColumns)
               nuisanceColumns.add(c);
            int gapJoints = 0;
            for (JointReadOnly spanningJoint : spanningJoints)
            {
               if (!jointToIndex.containsKey(spanningJoint)) // unfiltered => gap joint => marginalize it
               {
                  nuisanceColumns.add(indexProvider.getJointDoFIndices(spanningJoint)[0]);
                  gapJoints++;
               }
            }
            massMatrixNuisanceColumns = new int[nuisanceColumns.size()];
            for (int i = 0; i < massMatrixNuisanceColumns.length; i++)
               massMatrixNuisanceColumns[i] = nuisanceColumns.get(i);
            numNuisanceDoF = massMatrixNuisanceColumns.length; // 6 + gap joints

            LogTools.info("Joint-level KF process noise: Schur-complement path (sigma_tau = " + SIGMA_TAU + " N*m) over "
                          + n + " joints, marginalizing a " + numNuisanceDoF + "-DoF nuisance block (6-DoF floating base"
                          + (gapJoints > 0 ? " + " + gapJoints + " unfiltered gap joint(s))." : ")."));
         }
      }
      else
      {
         LogTools.info("Joint-level KF process noise: scalar CWNA fallback (no robot model provided).");
      }

      // 4) Precompute base->foot chains for the phase 2 stance anchoring measurement update
      if (baseIMU != null && feet != null)
      {
         for (RigidBodyBasics foot : feet)
         {
            FootAnchor fa = new FootAnchor(foot);
            fa.jac.setKinematicChain(baseIMU.getMeasurementLink(), foot);
            fa.jac.setJacobianFrame(baseIMU.getMeasurementFrame()); // express J in the same frame as the base gyro
            List<OneDoFJointBasics> leg = MultiBodySystemTools.filterJoints(fa.jac.getJointsFromBaseToEndEffector(), OneDoFJointBasics.class);

            fa.legJoints = leg.toArray(new OneDoFJointBasics[0]);
            fa.Jang.reshape(3, fa.legJoints.length);
            fa.qdCols = new int[fa.legJoints.length];
            fa.usable = true;
            for (int c = 0; c < fa.legJoints.length; c++)
            {
               Integer idx = jointToIndex.get(fa.legJoints[c]);
               if (idx == null)
               {
                  fa.usable = false;
                  fa.qdCols[c] = -1;
               }
               else
                  fa.qdCols[c] = n + idx;
            }
            footAnchors.add(fa);

         }
      }
      allocate();

      yoStateDimension.set(dim);
      yoNumberOfFilteredJoints.set(n);
      yoNumberOfIMUs.set(m);

      createJointYoVariables();
   }

   /**
    * Allocates the per-joint state / covariance-envelope YoVariables, one set per filtered joint, indexed by
    * the joint's state index so the per-tick update is a straight array write. Called once from the
    * constructor after the state layout (n) is fixed.
    */
   private void createJointYoVariables()
   {
      yoJointPosition = new YoDouble[n];
      yoJointVelocity = new YoDouble[n];
      yoJointPositionUpperBound = new YoDouble[n];
      yoJointPositionLowerBound = new YoDouble[n];
      yoJointVelocityUpperBound = new YoDouble[n];
      yoJointVelocityLowerBound = new YoDouble[n];
      for (var e : jointToIndex.entrySet())
      {
         int idx = e.getValue();
         String jointName = e.getKey().getName();
         yoJointPosition[idx] = new YoDouble("jointKF_q_" + jointName, registry);
         yoJointVelocity[idx] = new YoDouble("jointKF_qd_" + jointName, registry);
         yoJointPositionUpperBound[idx] = new YoDouble("jointKF_q_" + jointName + "_upperBound", registry);
         yoJointPositionLowerBound[idx] = new YoDouble("jointKF_q_" + jointName + "_lowerBound", registry);
         yoJointVelocityUpperBound[idx] = new YoDouble("jointKF_qd_" + jointName + "_upperBound", registry);
         yoJointVelocityLowerBound[idx] = new YoDouble("jointKF_qd_" + jointName + "_lowerBound", registry);
      }
   }

   /**
    * Publishes the per-joint estimate (q, qd) and its 1-sigma covariance envelope to the YoVariables. Reads
    * straight from x and P; allocation-free (only primitive .set()). The variance diagonal is clamped at 0
    * before the sqrt to stay finite through the numerical negatives a covariance can momentarily take.
    */
   private void updateJointYoVariables()
   {
      for (int i = 0; i < n; i++)
      {
         double q = x.get(i);
         double qd = x.get(n + i);
         double sigmaQ = Math.sqrt(Math.max(0.0, P.get(i, i)));
         double sigmaQd = Math.sqrt(Math.max(0.0, P.get(n + i, n + i)));
         yoJointPosition[i].set(q);
         yoJointVelocity[i].set(qd);
         yoJointPositionUpperBound[i].set(q + sigmaQ);
         yoJointPositionLowerBound[i].set(q - sigmaQ);
         yoJointVelocityUpperBound[i].set(qd + sigmaQd);
         yoJointVelocityLowerBound[i].set(qd - sigmaQd);
      }
   }

   // Mirrors AlphaComplimentaryFilter.createForKinematicsEstiamtor, works for the factory implementation
   public static JointLevelKFPreFilter createForKinematicsEstimator(SensorOutputMapReadOnly sensorOutputMap,
                                                                    StateEstimatorParameters stateEstimatorParameters,
                                                                    List<? extends IMUSensorReadOnly> imuProcessedOutputs,
                                                                    Collection<RigidBodyBasics> feet,
                                                                    RigidBodyBasics estimatorRootBody,
                                                                    double gravitationalAcceleration,
                                                                    BooleanProvider cancelGravityFromAccelerationMeasurement,
                                                                    double estimatorDT,
                                                                    YoRegistry parentRegistry)
   {
      //TODO: this function should be removed and the factory should handle this part.
      if (stateEstimatorParameters == null)
         throw new UnsupportedOperationException("default estimator parameters for this type of estimator are not added yet.");
      // imuProcessedOutputs / gravity / cancelGravity are unused: this filter resolves its IMUs from the
      // sensor map by the pair parameters' names (the same live IMU objects the alpha filter uses) and its
      // stance anchor is gyro-based. They are kept for signature parity with the alpha factory. The
      // registry, however, is now threaded through so the filter actually publishes on hardware.
      // estimatorRootBody (the estimator model's elevator) enables the mass-matrix process noise; null is a
      // supported degradation to the scalar-CWNA path.
      return new JointLevelKFPreFilter(sensorOutputMap,
                                       stateEstimatorParameters.getIMUBasedJointStateEstimatorParameters(),
                                       feet,
                                       estimatorRootBody,
                                       estimatorDT,
                                       parentRegistry);
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

   private void allocate()
   {
      int maxMeas = Math.max(n,3); // encoder update is the widest measurement
      x.reshape(dim, 1);
      xtmp.reshape(dim, 1);

      P.reshape(dim,dim);
      F.reshape(dim,dim);
      Q.reshape(dim,dim);
      PHt.reshape(dim,maxMeas);
      K.reshape(dim,maxMeas);
      KR.reshape(dim,maxMeas);
      S.reshape(maxMeas, maxMeas);
      Sinv.reshape(maxMeas,maxMeas);
      nu.reshape(maxMeas,1);
      Henc = new DMatrixRMaj(n,dim);
      Renc = new DMatrixRMaj(n,n);
      zEnc = new DMatrixRMaj(n,1);

      // Pre-size the Joseph-form scratch. IKH in particular MUST start at dim x dim: the first josephUpdate
      // does setIdentity(IKH) *before* multAdd reshapes it, so on a 0x0 IKH the identity is silently dropped
      // and the first covariance update becomes -KH instead of I-KH. Sizing both here also keeps their first
      // use from reallocating on the estimator thread.
      Ptmp.reshape(dim, dim);
      IKH.reshape(dim, dim);
      xBackup.reshape(dim, 1);
      PBackup.reshape(dim, dim);

      // Reused LU solver for the innovation-covariance inverse, warmed at the widest measurement (maxMeas).
      // The warm-up forces every internal buffer to its largest size now, so per-tick setA()/invert() reuse
      // them and never allocate — unlike CommonOps_DDRM.invert, which news up a decomposition every call for
      // any matrix wider than 5 (the n-joint encoder update).
      innovationSolver = LinearSolverFactory_DDRM.lu(maxMeas);
      innovationSolver.setA(CommonOps_DDRM.identity(maxMeas));

      // Schur-complement scratch + two Cholesky solvers, pre-warmed at their fixed sizes so the per-tick
      // setA()/solve()/invert() never allocate (chol decomposes in place and reuses its internal buffers once
      // warmed). Cholesky both exploits and enforces SPD-ness: a non-PD M_bb or Lambda (bad model state) fails
      // setA and the previous Q is kept instead of a garbage inverse entering the filter.
      if (massMatrixCalculator != null)
      {
         int nN = numNuisanceDoF;
         MNN.reshape(nN, nN);
         MNf.reshape(nN, n);
         Mff.reshape(n, n);
         MNNInvMNf.reshape(nN, n);
         Lambda.reshape(n, n);
         LambdaInv.reshape(n, n);
         LambdaInvSq.reshape(n, n);

         nuisanceMassMatrixSolver = LinearSolverFactory_DDRM.chol(nN);
         nuisanceMassMatrixSolver.setA(CommonOps_DDRM.identity(nN));
         // Warm the nuisance solver's multi-column solve buffers at RHS width n (M_NN X = M_Nf is N x n), so the
         // per-tick solve reuses them. The MNf scratch is still zero here — this is only a buffer warm-up.
         nuisanceMassMatrixSolver.solve(MNf, MNNInvMNf);

         schurSolver = LinearSolverFactory_DDRM.chol(n);
         schurSolver.setA(CommonOps_DDRM.identity(n));
      }

      for (int i = 0; i < 3; i++)
         anchorR.set(i, i, ANCHOR_VAR);

      buildConstantTransition();
      buildProcessNoise();
      buildEncoderModel();
      validateConstantModel();
   }

   /**
    * One-time (construction) finiteness check on the constant model matrices. These are built once and reused
    * every tick, so a single non-finite entry here — most plausibly an IMU that returns a non-finite
    * angular-velocity bias process-noise covariance while it is still booting — permanently poisons Q (or F)
    * and makes P go NaN on the first {@link #predict()}, with no recovery. Logged as an error naming the
    * matrix so this shows up clearly at estimator start instead of as a silent all-NaN filter. Runs at
    * construction only; the string work here is not on the estimator hot path.
    */
   private void validateConstantModel()
   {
      if (containsNonFinite(F))
         LogTools.error("JointLevelKFPreFilter transition matrix F is non-finite at construction.");
      if (containsNonFinite(Q))
         LogTools.error("JointLevelKFPreFilter process-noise Q is non-finite at construction — check the per-IMU "
                        + "angular-velocity bias process-noise covariances (see buildProcessNoise).");
      if (containsNonFinite(Henc))
         LogTools.error("JointLevelKFPreFilter encoder Jacobian Henc is non-finite at construction.");
      if (containsNonFinite(Renc))
         LogTools.error("JointLevelKFPreFilter encoder noise Renc is non-finite at construction.");
   }

   private void buildConstantTransition()
   {
      CommonOps_DDRM.setIdentity(F);
      for (int i = 0; i < n; i++)
         F.set(i, n + i, dt); // q_{k+1} = q_k + dt * qd_k ; qd and bias are constant with noise
   }

   private void buildProcessNoise()
   {
      Q.zero();
      double sa2 = SIGMA_ACCEL * SIGMA_ACCEL;
      double dt2 = dt * dt;
      double dt3 = dt2 * dt;

      // Scalar-CWNA joint blocks first: this is both the no-model path and the fallback the mass-matrix
      // update leaves in place if its very first computation fails (e.g. model not yet in a valid pose).
      for (int i = 0; i < n; i++)
      {
         Q.set(i, i, sa2 * dt3/3.0); // CT white noise accel block, per joint
         Q.set(i, n + i, sa2 * dt2 / 2.0); // q - qd cross term
         Q.set(n + i, i, sa2 * dt2 / 2.0);
         Q.set(n + i, n + i, sa2 * dt);
      }
      for (var e : imuToOrdinal.entrySet()) // bias random walk, per-IMU process noise
      {
         e.getKey().getAngularVelocityBiasProcessNoiseCovariance(Rimu);
         // Do not let a non-finite covariance (e.g. an IMU still booting at construction) poison Q: a single
         // NaN here spreads to P on the first predict and never recovers. Skip this IMU's bias random-walk
         // instead, and name it so the offender is obvious in the log.
         if (containsNonFinite(Rimu))
         {
            LogTools.error("Non-finite angular-velocity bias process-noise covariance from IMU " + e.getKey().getSensorName()
                           + " at construction; skipping its Q contribution (its bias random-walk is disabled) so Q stays finite.");
            continue;
         }
         int col = 2 * n + 3 * e.getValue();
         for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
               Q.set(col + i, col + j, Q.get(col + i, col + j) + dt * Rimu.get(i,j));
      }

      // Overwrites the joint (q, qd) blocks with the mass-matrix Qa if a model is available; no-op otherwise.
      updateProcessNoiseFromMassMatrix();
   }

   /**
    * Rebuilds the joint-space Van Loan blocks of Q from the Schur-complement acceleration noise
    * Qa = sigma_tau^2 Lambda(q)^-2 (SPEC §3.2): an unmodeled joint torque w_tau propagates through the
    * floating-base dynamics, and eliminating the (unforced) nuisance acceleration from the perturbed EOM gives
    * delta_qddot = Lambda^-1 w_tau with Lambda = M_ff - M_jN M_NN^-1 M_Nj the Schur complement of the nuisance
    * block (nuisance = 6-DoF base + any unfiltered gap joints; see the field comment). For the real gapless
    * topology the nuisance is exactly the 6-DoF base and this is the SPEC's Lambda = M_jj - M_jb M_bb^-1 M_bj.
    * For scalar Sigma_tau = sigma_tau^2 I and symmetric Lambda this is sigma_tau^2 Lambda^-2. Called once from
    * {@link #buildProcessNoise()} and then at the top of every {@link #predict()}, because M(q) — read from the
    * live model joints, whose frames the estimator refreshes each tick — is configuration-dependent. Only the
    * (q, qd) blocks are touched; the bias random-walk block and the zero joint/bias coupling are left exactly
    * as {@link #buildProcessNoise()} made them.
    *
    * <p>Numerical failure (non-finite M, Cholesky rejection of a non-PD M_NN or Lambda, non-finite
    * intermediate) leaves the previous Q in place — which at worst is the scalar-CWNA build — and warns once,
    * identical to the Rev. 1 fallback philosophy. Allocation-free: the calculator, all scratch matrices, and
    * both Cholesky solvers are pre-sized/warmed in {@link #allocate()}.</p>
    *
    * <p>TODO(retune): SPEC §8 — Lambda^-2 ⪰ M_jj^-2, so the effective Qa grows at fixed sigma_tau relative to
    * the Rev. 1 locked-base map. SIGMA_TAU must be retuned against quiet-standing and walking NIS by a human;
    * do NOT carry the Rev. 1 value over as if it were still calibrated.</p>
    */
   private void updateProcessNoiseFromMassMatrix()
   {
      if (massMatrixCalculator == null)
         return;

      massMatrixCalculator.reset();
      // Full mass matrix over {6-DoF base} ∪ {filtered joints}, ordered by the calculator's index provider.
      DMatrixRMaj massMatrix = massMatrixCalculator.getMassMatrix();
      if (containsNonFinite(massMatrix))
      {
         warnMassMatrixFailureOnce("non-finite mass matrix");
         return;
      }

      // Extract the blocks by resolved column index (never assume ordering — SPEC §3.2). M is symmetric, so
      // M_jN = M_Nj^T and we only need M_NN, M_Nf, M_ff. M_ff is read in filter joint state order (row i =
      // filtered joint i), so Lambda and Lambda^-2 come out already in state order — no permutation in the fill.
      int nN = numNuisanceDoF;
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

      // X = M_NN^-1 M_Nf via Cholesky (rejects a non-PD nuisance block before it can enter the Schur complement).
      if (!nuisanceMassMatrixSolver.setA(MNN))
      {
         warnMassMatrixFailureOnce("nuisance mass-matrix block M_NN not positive definite");
         return;
      }
      nuisanceMassMatrixSolver.solve(MNf, MNNInvMNf); // X (Nxn); solve leaves MNf unmodified
      if (containsNonFinite(MNNInvMNf))
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
      symmetrize(Lambda);

      if (!schurSolver.setA(Lambda)) // Cholesky: rejects a non-PD Schur complement before it can enter Q
      {
         warnMassMatrixFailureOnce("Schur complement Lambda not positive definite");
         return;
      }
      schurSolver.invert(LambdaInv);
      if (containsNonFinite(LambdaInv))
      {
         warnMassMatrixFailureOnce("non-finite Schur-complement inverse (near-singular Lambda)");
         return;
      }
      CommonOps_DDRM.mult(LambdaInv, LambdaInv, LambdaInvSq); // Lambda^-2; Lambda symmetric => Lambda^-1 Lambda^-T = Lambda^-2

      double st2 = SIGMA_TAU * SIGMA_TAU;
      double dt2 = dt * dt;
      double dt3 = dt2 * dt;
      for (int i = 0; i < n; i++)
      {
         for (int j = 0; j < n; j++)
         {
            // Symmetrized read (SPEC §3.4): the Cholesky inverse and its square are symmetric only to
            // round-off, and Q must stay exactly symmetric or the Joseph update slowly loses P's symmetry.
            double qa = st2 * 0.5 * (LambdaInvSq.get(i, j) + LambdaInvSq.get(j, i));
            Q.set(i, j, qa * dt3 / 3.0);     // q-q block
            Q.set(i, n + j, qa * dt2 / 2.0); // q-qd cross term
            Q.set(n + i, j, qa * dt2 / 2.0);
            Q.set(n + i, n + j, qa * dt);    // qd-qd block
         }
      }
   }

   /** In-place symmetrization A <- 0.5 (A + A^T). Allocation-free. */
   private static void symmetrize(DMatrixRMaj a)
   {
      for (int i = 0; i < a.numRows; i++)
      {
         for (int j = i + 1; j < a.numCols; j++)
         {
            double avg = 0.5 * (a.get(i, j) + a.get(j, i));
            a.set(i, j, avg);
            a.set(j, i, avg);
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

   private void buildEncoderModel()
   {
      Henc.zero();
      for (int i = 0; i < n; i++)
         Henc.set(i, i, 1.0);
      Renc.zero();
      for (int i = 0; i < n; i++)
         Renc.set(i, i, ENCODER_VAR);
   }

   /**
    * Installs the optional on-ground gate: while {@code onGround} does not read true, {@link #initialize()}
    * stays deferred (the filter exports NaN joint states and zero bias, so consumers fall back to the raw
    * sensors). Pass {@code null} to disable gating (the default — initialize as soon as boot data is finite).
    * The gate is debounced internally over {@link #ON_GROUND_INIT_DEBOUNCE}; see that constant for why a
    * single-tick check is not enough. Called once at wiring time by the consumer that owns the
    * contact-probability signal; never on the estimator hot path.
    */
   public void setInitializationGate(BooleanSupplier onGround)
   {
      this.onGroundGate = onGround;
      this.consecutiveOnGroundTicks = 0;
      yoWaitingForGroundContact.set(onGround != null && !initialized);
   }

   /**
    * True when the filter is clear to seed this tick: no gate wired, or the gate has read true for
    * {@link #requiredOnGroundTicks} consecutive attempts (both feet firmly in contact). Advances/resets the
    * debounce counter as a side effect, so it is called exactly once per {@link #initialize()} attempt.
    */
   private boolean readyToInitialize()
   {
      if (onGroundGate == null)
         return true;
      if (onGroundGate.getAsBoolean())
         consecutiveOnGroundTicks++;
      else
         consecutiveOnGroundTicks = 0;
      boolean ready = consecutiveOnGroundTicks >= requiredOnGroundTicks;
      yoWaitingForGroundContact.set(!ready);
      return ready;
   }

   @Override
   public void initialize()
   {
      // Do not seed the filter until the robot is firmly on the ground: the exported base-IMU gyro bias is
      // unobservable while hanging (its only anchor is the phase-2 stance update, gated off when no foot is
      // trusted), so seeding then lets the bias wander and the downstream InEKF integrates that into a
      // rotating/glitching base. Staying uninitialized exports NaN/zero, which the consumers treat as
      // "fall back to the raw gyro/sensors". No-op when no gate is wired (unit tests / non-invariant pipelines).
      if (!readyToInitialize())
         return;

      // Refuse to latch non-finite boot data: a KF permanently latches NaN (predict/Joseph spread it
      // through x and P with no recovery when sensors come good), so stay uninitialized and retry next
      // tick instead. While uninitialized, getEstimatedJoint* return NaN — which the consumers
      // (JointStateUpdater / InvariantMainStateEstimator.updateJoints) treat as "fall back to the raw
      // sensor" — and the bias getters return zero. Same fail-soft behavior as the alpha filter.
      for (var e : jointToIndex.entrySet())
      {
         if (!Double.isFinite(sensorMap.getOneDoFJointOutput(e.getKey()).getPosition()))
         {
            warnNonFiniteInputOnce("joint position of " + e.getKey().getName() + " at initialization");
            return;
         }
      }
      x.zero();
      for (var e : jointToIndex.entrySet())
         x.set(e.getValue(), sensorMap.getOneDoFJointOutput(e.getKey()).getPosition());
      P.zero();
      for (int i = 0; i < n; i++) P.set(i, i, INIT_POS_VAR);
      for (int i = n; i < 2 * n; i++) P.set(i, i, INIT_VEL_VAR);
      for (int i = 2 * n; i < dim; i++) P.set(i, i, INIT_BIAS_VAR);
      initialized = true;
      yoInitialized.set(true);
      yoWaitingForGroundContact.set(false);
   }

   // ================================ Phase 1 ================================

   @Override
   public void computeJointState()
   {
      // Phase 1: joint predict + encoder/pair-gyro measurement updates go here.
      if (!initialized)
      {
         initialize();
         if (!initialized)
            return; // boot data not valid yet; consumers keep falling back to raw sensors until it is
      }

      predict();
      warnIfNonFiniteState("predict", -1);

      // update encoder states; skipped wholesale if any encoder reads non-finite (boot transient)
      int row = 0;
      boolean encodersValid = true;
      for (OneDoFJointBasics j : jointToIndex.keySet())
      {
         double q = sensorMap.getOneDoFJointOutput(j).getPosition();
         if (!Double.isFinite(q))
         {
            encodersValid = false;
            if (!warnedNonFiniteInput)
               warnNonFiniteInputOnce("joint position of " + j.getName());
         }
         zEnc.set(row++, 0, q);
      }
      if (encodersValid)
      {
         josephUpdate(Henc, zEnc, Renc, "encoder");
         warnIfNonFiniteState("encoderUpdate", -1);
      }

      for (int i = 0; i < pairs.size(); i++)
      {
         pairGyroUpdate(pairs.get(i));
         warnIfNonFiniteState("pairGyroUpdate", i);
      }

      updateJointYoVariables();
   }

   /** EKF time update in isolation: x <- F x, P <- F P F^T + Q(q). Package-private so tests can drive it alone. */
   void predict()
   {
      // Q is state-dependent on the mass-matrix path (Qa = sigma_tau^2 M(q)^-2), so refresh it from the
      // model's current configuration before propagating the covariance. No-op on the scalar fallback path.
      updateProcessNoiseFromMassMatrix();
      CommonOps_DDRM.mult(F, x, xtmp);
      x.set(xtmp);
      CommonOps_DDRM.mult(F, P, Ptmp);
      CommonOps_DDRM.multTransB(Ptmp, F, P); // FPF^T term
      CommonOps_DDRM.addEquals(P, Q);
   }

   private void pairGyroUpdate(Pair p)
   {
      buildPairMeasurement(p);
      // Guard z (IMU gyro not booted yet), H (Jacobian built from a model whose joints were set from
      // non-finite raw sensors by the consumer's own fallback), AND R3 (built from the IMU gyro measurement
      // covariances and the frame rotations — previously unchecked, so a non-finite covariance flowed straight
      // into the update). One bad update would latch NaN into x and P permanently, so skip this pair for the tick.
      if (containsNonFinite(zMeas) || containsNonFinite(H) || containsNonFinite(R3))
      {
         if (!warnedNonFiniteInput)
            warnNonFiniteInputOnce("pair gyro measurement " + p.parent.getSensorName() + " -> " + p.child.getSensorName());
         return;
      }
      josephUpdate(H, zMeas, R3, "pairGyro");
   }

   /**
    * Fills the pair-gyro measurement model (H, zMeas, R3) for one IMU pair, without applying the update.
    * Split out of {@link #pairGyroUpdate} so tests can inspect the measurement model (mirrors the reference
    * implementation's build_measurement / build_H seam).
    */
   private void buildPairMeasurement(Pair p)
   {
      // Frames are already current: the estimator calls rootBody.updateFramesRecursively() every tick (this
      // filter runs inside that tick and reads the same joints the alpha estimator does). Re-updating them
      // per joint here is redundant AND allocates ~64 B/joint inside MovingReferenceFrame.update() — garbage
      // on the estimator thread that the alpha filter never generated.
      p.jac.reset();
      CommonOps_DDRM.extract(p.jac.getJacobianMatrix(), 0, 3, 0, p.qdCols.length, p.Jang, 0, 0); //angular part

      // z = omega_child - omega_parent, expressed in the Jacobian frame (same construction as the alpha estimator)
      fvA.setToZero(p.child.getMeasurementFrame());
      fvA.set(p.child.getAngularVelocityMeasurement());
      fvA.changeFrame(p.jac.getJacobianFrame());

      fvB.setToZero(p.parent.getMeasurementFrame());
      fvB.set(p.parent.getAngularVelocityMeasurement());
      fvB.changeFrame(p.jac.getJacobianFrame());
      fvA.sub(fvB);
      fvA.get(zMeas);

      // H (3 x dim) ; velocity block = scattered J columns, bias blocks = +R_child, -R_parent; q block = 0
      H.reshape(3, dim);
      H.zero();
      for (int c = 0; c < p.qdCols.length; c++)
         for (int r = 0; r < 3; r++)
            H.set(r, p.qdCols[c], p.Jang.get(r,c));
      packRotationToJacFrame(p.child, p.jac.getJacobianFrame(), rot3);
      insertScaledInto(rot3, +1.0, H, 0, p.childBias);
      packRotationToJacFrame(p.parent, p.jac.getJacobianFrame(), rot3);
      insertScaledInto(rot3, -1.0, H, 0, p.parentBias);
      // q columns left 0: dJ/dq * qd neglected (encoder has pinned q)
      // R = R_child Sigma_child R_child^T + R_parent Sigma_parent R_parent^T (L Sigma L^T), where Sigma is each
      // IMU's angular-velocity MEASUREMENT noise covariance (the per-IMU Mahony filter output covariance, R_omega
      // in the write-up). This is deliberately NOT getAngularVelocityBiasProcessNoiseCovariance — that is the
      // bias random-walk intensity used in buildProcessNoise, typically orders of magnitude smaller. Using it
      // here made the pair-gyro update drastically over-confident.
      R3.zero();
      packRotationToJacFrame(p.child, p.jac.getJacobianFrame(), rot3);
      p.child.getAngularVelocityNoiseCovariance(Rimu);
      congruenceAdd(rot3, Rimu, R3);
      packRotationToJacFrame(p.parent, p.jac.getJacobianFrame(), rot3);
      p.parent.getAngularVelocityNoiseCovariance(Rimu);
      congruenceAdd(rot3, Rimu, R3);
   }

   // ================================ Phase 2 ================================

   @Override
   public void computeImuBiases(List<RigidBodyBasics> trustedFeet)
   {
      if (!initialized || trustedFeet.isEmpty())
         return; // no absolute reference at the current tick; common mode bias gauge stays free
      for (int i = 0; i < footAnchors.size(); i++)
      {
         FootAnchor fa = footAnchors.get(i);
         if (fa.usable && trustedFeet.contains(fa.foot))
         {
            stanceAnchorUpdate(fa);
            warnIfNonFiniteState("stanceAnchor", i);
         }
      }
   }

   private void stanceAnchorUpdate(FootAnchor fa)
   {
      // Frames already current (see pairGyroUpdate); do not re-update per joint (redundant + allocates).
      CommonOps_DDRM.extract(fa.jac.getJacobianMatrix(), 0, 3, 0, fa.qdCols.length, fa.Jang, 0, 0);

      // Measurement: base IMU gyro, already in measurement frame
      Vector3DReadOnly w = baseIMU.getAngularVelocityMeasurement();
      zAnchor.set(0, 0, w.getX());
      zAnchor.set(1,0, w.getY());
      zAnchor.set(2,0, w.getZ());

      // Model with no contact twist: (omega_foot = 0) => z = -J_ang * qd_leg + b_base
      // Verify with convention before trusting, sign of J_ang and the
      // assumption that the stance foot is non-rotating is what makes this work.
      H.reshape(3,dim);
      H.zero();
      for (int c = 0; c < fa.qdCols.length; c++)
         for (int r = 0; r < 3; r++)
            H.set(r, fa.qdCols[c], -fa.Jang.get(r,c));
      H.set(0, baseBiasCol, 1.0);
      H.set(1, baseBiasCol + 1, 1.0);
      H.set(2, baseBiasCol + 2, 1.0);

      if (containsNonFinite(zAnchor) || containsNonFinite(H))
      {
         if (!warnedNonFiniteInput)
            warnNonFiniteInputOnce("stance anchor at " + fa.foot.getName() + " (base IMU " + baseIMU.getSensorName() + ")");
         return;
      }
      josephUpdate(H, zAnchor, anchorR, "stanceAnchor");
   }

   // ================================ KF core ================================
   // Sequential Joseph form measurement update for any matrix of dimension k.
   // Package-private so tests can drive a single measurement update against a reference KF.
   void josephUpdate(DMatrixRMaj Hm, DMatrixRMaj zm, DMatrixRMaj Rm)
   {
      josephUpdate(Hm, zm, Rm, "test"); // backward-compatible overload for the package-private unit tests
   }

   /**
    * Sequential Joseph-form measurement update, hardened against latching NaN. {@code label} names the
    * calling update ("encoder" / "pairGyro" / "stanceAnchor") so a skip or rollback identifies its source in
    * the log. Two extra guards over a textbook update:
    * <ul>
    *   <li>after the innovation inverse, if S^-1 is non-finite (S was only <em>near</em>-singular, so the LU
    *   {@code setA} did not reject it but {@code invert} overflowed), the update is skipped <em>before</em> x
    *   or P are touched;</li>
    *   <li>if the completed update still leaves x or P non-finite (e.g. a finite-but-huge intermediate
    *   overflowed to +/-Inf), the pre-update mean and covariance are restored from the backups.</li>
    * </ul>
    * Both leave the filter in its prior valid state rather than propagating NaN forever.
    */
   void josephUpdate(DMatrixRMaj Hm, DMatrixRMaj zm, DMatrixRMaj Rm, String label)
   {
      int k = Hm.getNumRows();
      PHt.reshape(dim, k);
      S.reshape(k, k);
      Sinv.reshape(k,k);
      nu.reshape(k,1);

      CommonOps_DDRM.multTransB(P, Hm, PHt);
      CommonOps_DDRM.mult(Hm, PHt, S);
      CommonOps_DDRM.addEquals(S, Rm); // innovation covariance S = H P H^T + R (the +R was previously missing,
                                       // which made the gain over-confident and could grow the covariance)
      // Allocation-free inverse via the pre-warmed solver (see allocate()). setA decomposes S in place;
      // invert writes S^-1 into Sinv. Neither allocates because S is never wider than the warm-up size.
      if (!innovationSolver.setA(S))
      {
         if (!warnedNonFiniteInput)
            warnNonFiniteInputOnce("singular innovation covariance in the " + label + " update");
         return;
      }
      innovationSolver.invert(Sinv);
      // setA only rejects an *exactly* singular S; a near-singular S passes and invert overflows to Inf/NaN.
      // Catch it here, before x/P are modified, so the bad inverse cannot enter the state.
      if (containsNonFinite(Sinv))
      {
         if (!warnedNonFiniteInput)
            warnNonFiniteInputOnce("non-finite innovation inverse (near-singular S) in the " + label + " update");
         return;
      }

      // Snapshot for rollback: everything below writes x then P in place.
      xBackup.set(x);
      PBackup.set(P);

      K.reshape(dim, k);
      CommonOps_DDRM.mult(PHt, Sinv, K);
      CommonOps_DDRM.mult(Hm, x, nu);
      CommonOps_DDRM.changeSign(nu);
      CommonOps_DDRM.addEquals(nu, zm);
      CommonOps_DDRM.multAdd(K, nu, x);

      // Full Joseph form update
      CommonOps_DDRM.setIdentity(IKH);
      CommonOps_DDRM.multAdd(-1.0, K, Hm, IKH); // I - KH
      CommonOps_DDRM.mult(IKH, P, Ptmp);
      CommonOps_DDRM.multTransB(Ptmp, IKH, P);
      KR.reshape(dim, k);
      CommonOps_DDRM.mult(K, Rm, KR);
      CommonOps_DDRM.multAddTransB(KR, K, P); // + K R K^T

      if (containsNonFinite(x) || containsNonFinite(P))
      {
         x.set(xBackup);
         P.set(PBackup);
         if (!warnedNonFiniteInput)
            warnNonFiniteInputOnce("the " + label + " update produced a non-finite state; rolled back to the prior estimate");
      }
   }

   /**
    * One-shot triage hook: the first time the filter's state goes non-finite, logs which stage produced it
    * (predict / encoderUpdate / pairGyroUpdate #i / stanceAnchor #i) and then stays quiet. Allocation-free
    * until it fires (the message is only built on that single occurrence); the finiteness scans are O(dim^2)
    * but run only until the first report. With the input/inverse/rollback guards above this should not fire —
    * if it does, its stage name is the exact place NaN enters, which is what to chase next.
    */
   private void warnIfNonFiniteState(String stage, int index)
   {
      if (nonFiniteStateReported)
         return;
      if (containsNonFinite(x) || containsNonFinite(P))
      {
         nonFiniteStateReported = true;
         LogTools.error("JointLevelKFPreFilter state first went non-finite after stage '" + stage + "'"
                        + (index >= 0 ? " #" + index : "") + " [x non-finite=" + containsNonFinite(x)
                        + ", P non-finite=" + containsNonFinite(P) + "]. This is where the NaN enters.");
      }
   }


   private void packRotationToJacFrame(IMUSensorReadOnly imu, ReferenceFrame jacFrame, DMatrixRMaj out)
   {
      imu.getMeasurementFrame().getTransformToDesiredFrame(tmpTransform, jacFrame);
      RotationMatrixReadOnly r = tmpTransform.getRotation();
      out.reshape(3,3);

      set_matrix(out, r);
   }

   public static void set_matrix(DMatrixRMaj out, RotationMatrixReadOnly r)
   {
      out.set(0, 0, r.getM00());
      out.set(0, 1, r.getM01());
      out.set(0, 2, r.getM02());

      out.set(1, 0, r.getM10());
      out.set(1, 1, r.getM11());
      out.set(1, 2, r.getM12());

      out.set(2, 0, r.getM20());
      out.set(2, 1, r.getM21());
      out.set(2, 2, r.getM22());
   }

   private void congruenceAdd(DMatrixRMaj L, DMatrixRMaj Sigma, DMatrixRMaj accum)
   {
      CommonOps_DDRM.mult(L, Sigma, tmp3a);
      CommonOps_DDRM.multTransB(tmp3a, L, tmp3b);
      CommonOps_DDRM.addEquals(accum, tmp3b);
   }

   private static void insertScaledInto(DMatrixRMaj src, double scale, DMatrixRMaj dst, int row0, int col0)
   {
      for (int i = 0; i < src.getNumRows(); i++)
         for (int j = 0; j < src.getNumCols(); j++)
            dst.set(row0 + i, col0 + j, scale * src.get(i,j));
   }

   /** True if any entry of the matrix is NaN or infinite. Allocation-free; O(elements). */
   private static boolean containsNonFinite(DMatrixRMaj mat)
   {
      for (int i = 0; i < mat.getNumElements(); i++)
      {
         if (!Double.isFinite(mat.get(i)))
            return true;
      }
      return false;
   }

   /**
    * Logs the FIRST non-finite input source ever seen, once — identifying which sensor is late in the
    * boot sequence — then stays silent (this runs on the estimator thread; the string concat only
    * happens on that single occurrence).
    */
   private void warnNonFiniteInputOnce(String source)
   {
      if (warnedNonFiniteInput)
         return;
      warnedNonFiniteInput = true;
      LogTools.warn("Non-finite input to JointLevelKFPreFilter; first offender: " + source
            + ". Affected updates are skipped and consumers fall back to raw sensors / zero bias.");
   }

   private static IMUSensorReadOnly findIMU(SensorOutputMapReadOnly map, String name)
   {
      for (int i = 0; i < map.getIMUOutputs().size(); i++)
      {
         IMUSensorReadOnly s = map.getIMUOutputs().get(i);
         if (s.getSensorName().equals(name)) return s;
      }
      return null;
   }

   @Override
   public boolean containsJoint(OneDoFJointBasics joint)
   {
      return jointToIndex.containsKey(joint);
   }

   @Override
   public double getEstimatedJointPosition(OneDoFJointBasics joint)
   {
      Integer idx = jointToIndex.get(joint);
      return (idx == null || !initialized) ? Double.NaN : x.get(idx);
   }

   @Override
   public double getEstimatedJointVelocity(OneDoFJointBasics joint)
   {
      Integer idx = jointToIndex.get(joint);
      return (idx == null || !initialized) ? Double.NaN : x.get(n + idx);
   }

   @Override
   public boolean hasCovariance()
   {
      return initialized;
   }

   @Override
   public void packPositionCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack)
   {
      packCov(joints, 0, fallbackVariance, toPack);
   }

   @Override
   public void packVelocityCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack)
   {
      packCov(joints, n, fallbackVariance, toPack);
   }

   private void packCov(OneDoFJointBasics[] joints, int blockOffset, double fallbackVariance, DMatrixRMaj toPack)
   {
      int mm = joints.length;
      toPack.reshape(mm, mm);
      toPack.zero();
      for (int a = 0; a < mm; a++)
      {
         Integer ia = jointToIndex.get(joints[a]);
         if (ia == null)
         {
            toPack.set(a, a, fallbackVariance);
            continue;
         }
         for (int b = 0; b < mm; b++)
         {
            Integer ib = jointToIndex.get(joints[b]);
            if (ib != null)
               toPack.set(a, b, P.get(blockOffset + ia, blockOffset + ib)); // adding the off diagonals for the cross-covariances
         }
      }
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      Integer ord = imuToOrdinal.get(imu);
      if (ord == null || !initialized)
      {
         // IMU not in the filter's state (e.g. the primary pelvis IMU when it isn't a pair member),
         // or the filter hasn't initialized yet: report zero bias. MUST return here — falling through
         // would unbox a null ord in "3 * ord" and NPE every tick on the estimator thread.
         biasOut.setToZero(imu.getMeasurementFrame());
         return biasOut;
      }
      int col = 2 * n + 3 * ord;
      double bx = x.get(col);
      double by = x.get(col + 1);
      double bz = x.get(col + 2);
      if (!Double.isFinite(bx) || !Double.isFinite(by) || !Double.isFinite(bz))
      {
         // Fail soft like the alpha filter's bias provider: never export a non-finite bias. This is
         // consumed directly by the InEKF's predict (gyro - bias -> R*exp(phi*dt)), which throws
         // NotARotationMatrixException on NaN — a hardware-only crash the joint NaN-fallback can't catch.
         warnNonFiniteInputOnce("bias state of " + imu.getSensorName());
         biasOut.setToZero(imu.getMeasurementFrame());
         return biasOut;
      }
      biasOut.setIncludingFrame(imu.getMeasurementFrame(), bx, by, bz);
      return biasOut;
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      getAngularVelocityBiasInIMUFrame(imu);
      biasOut.changeFrame(ReferenceFrame.getWorldFrame());
      return biasOut;
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      biasOut.setToZero(imu.getMeasurementFrame());
      return biasOut;
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      biasOut.setToZero(ReferenceFrame.getWorldFrame());
      return biasOut;
   }

   // ================================ Test surface ================================
   // Package-private read/seed hooks used only by the unit tests in this package (mirroring the reference
   // implementation's pure-function seams). Not part of the public API and never called on the estimator
   // thread. Getters return fresh copies so a test cannot mutate filter internals.

   int getStateDimension()          { return dim; }
   int getNumberOfFilteredJoints()  { return n; }
   int getNumberOfIMUs()            { return m; }
   int getNumberOfPairs()           { return pairs.size(); }

   /** State index of the given joint's position entry (its velocity entry is this + n); -1 if not filtered. */
   int getJointStateIndex(OneDoFJointBasics joint) { Integer i = jointToIndex.get(joint); return i == null ? -1 : i; }
   List<OneDoFJointBasics> getFilteredJointsInStateOrder() { return new ArrayList<>(jointToIndex.keySet()); }

   DMatrixRMaj getStateVector()      { return x.copy(); }     // x = [q ; q_dot ; b_omega]
   DMatrixRMaj getCovariance()       { return P.copy(); }
   DMatrixRMaj getTransitionMatrix() { return F.copy(); }
   DMatrixRMaj getProcessNoise()     { return Q.copy(); }

   /** True when Q's joint blocks come from Qa = sigma_tau^2 M(q)^-2 (a robot model was provided). */
   boolean isUsingMassMatrixProcessNoise() { return massMatrixCalculator != null; }
   /** Recomputes the mass-matrix Q blocks from the model's current configuration (no-op on the scalar path). */
   void updateProcessNoiseFromMassMatrixForTest() { updateProcessNoiseFromMassMatrix(); }
   DMatrixRMaj getEncoderJacobian()  { return Henc.copy(); }
   DMatrixRMaj getEncoderNoise()     { return Renc.copy(); }

   /** Overwrites the mean and covariance (and marks initialized) so tests can drive predict()/josephUpdate() from a known prior. */
   void setStateForTest(DMatrixRMaj xPrior, DMatrixRMaj pPrior)
   {
      x.set(xPrior);
      P.set(pPrior);
      initialized = true;
   }

   /** Builds the pair-gyro measurement (H, z, R) for the given pair without applying the update, for inspection. */
   void buildPairMeasurementForTest(int pairIndex) { buildPairMeasurement(pairs.get(pairIndex)); }
   DMatrixRMaj getMeasurementJacobian() { return H.copy(); }     // last H built (3 x dim)
   DMatrixRMaj getMeasurementResidual() { return zMeas.copy(); } // last z built (3 x 1)
   DMatrixRMaj getMeasurementNoise()    { return R3.copy(); }    // last R built (3 x 3)
   int[] getPairVelocityColumns(int pairIndex) { return pairs.get(pairIndex).qdCols.clone(); }
   int getPairParentBiasColumn(int pairIndex)  { return pairs.get(pairIndex).parentBias; }
   int getPairChildBiasColumn(int pairIndex)   { return pairs.get(pairIndex).childBias; }

   // ================================ Structure holders ================================
   private static final class Pair
   {
      final IMUSensorReadOnly parent, child;
      final GeometricJacobianCalculator jac = new GeometricJacobianCalculator();
      OneDoFJointBasics[] chainJoints;
      int[] qdCols;
      int parentBias, childBias;
      final DMatrixRMaj Jang = new DMatrixRMaj(3,1);
      Pair(IMUSensorReadOnly parent, IMUSensorReadOnly child)
      {
         this.parent = parent;
         this.child = child;
      }
   }

   private static final class FootAnchor
   {
      final RigidBodyBasics foot;
      final GeometricJacobianCalculator jac = new GeometricJacobianCalculator();
      OneDoFJointBasics[] legJoints;
      int[] qdCols;
      boolean usable; // false if any base-> foot joint is not in the state
      final DMatrixRMaj Jang = new DMatrixRMaj(3,1);
      FootAnchor(RigidBodyBasics foot)
      {
         this.foot = foot;
      }
   }

}

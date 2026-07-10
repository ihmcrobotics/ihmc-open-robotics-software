package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.dense.row.decomposition.chol.CholeskyDecompositionInner_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.dense.row.linsol.chol.LinearSolverChol_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
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
 * per-IMU biases in state and stance-foot gauge anchors for the absolute base bias. When a robot model is
 * provided, the joint process noise is the mass-matrix-induced Qa = sigma_tau^2 M(q)^-2 of the write-up
 * (eqs. (10)-(12)), recomputed each predict; otherwise a scalar-CWNA diagonal fallback.
 *
 * <p><b>Do not share an instance between pipelines</b> (the filter holds its covariance).</p>
 *
 * <p><b>Rev. 2 measurement model (SPEC §5-§6).</b> The gyro measurements are applied as ONE stacked Joseph
 * update per tick over z_g = [pair diffs ; active stance anchors], not the Rev. 1 per-pair sequential updates.
 * Its noise R_g = L Sigma L^T + Sigma_eps carries the exact shared-IMU cross-covariances (biases and gyro white
 * noise both enter through the same edge-incidence operator L, whose blocks also ARE the bias columns of H_g).
 * The stance anchors are merged into this phase-1 update using the previous tick's trusted-feet set;
 * {@link #computeImuBiases} is reduced to caching that set. See jointKF_derivation.md §5-§7.</p>
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
   // N*m unmodeled-torque STD for the mass-matrix path: Qa = Lambda_eff^-1 diag(sigma_tau,i^2) Lambda_eff^-T.
   // SIGMA_TAU is now only the FALLBACK STD used for a joint whose effort limit is absent/non-finite; the live
   // per-joint STD is sigma_tau,i = ALPHA * tau_max,i (see ALPHA and sigmaTauPerJoint). Kept at 5 N*m as the
   // Rev.2 interim (Lambda^-2 ⪰ M_jj^-2 inflated Qa vs. the Rev.1 locked-base map). TODO(retune) with ALPHA.
   private static final double SIGMA_TAU = 5.0;
   // Per-joint unmodeled-torque STD as a fraction of the joint's effort limit: sigma_tau,i = ALPHA * tau_max,i
   // (Part B item 3), tau_max,i from OneDoFJointReadOnly.getEffortLimitUpper(). Physically the unmodeled torque
   // scales with the actuator's own torque capacity, so this is a far better prior than a single scalar across a
   // hip (217 N*m) and a wrist (a few N*m). TODO(retune): ALPHA=0.15 is an interim, uncalibrated value — the
   // human calibrates it against quiet-standing then walking NIS, gyro block and encoder block separately.
   private static final double ALPHA = 0.15;
   // TRIPWIRE ONLY (Part B item 2 — no longer a scaler). Physical ceiling on the per-joint acceleration
   // process-noise VARIANCE (~ (30 rad/s^2)^2). With the reflected-rotor-inertia floor on Lambda_eff (item 1),
   // max diag(Qa) must sit far below this; if it ever would not, that is a model/config regression to SURFACE,
   // not to hide. When max diag(Qa) > QA_MAX we warn once (naming the argmax joint) and count it in
   // jointKFQaCapWouldBindCount, but we DO NOT rescale — the old uniform CommonOps_DDRM.scale coupled one
   // joint's outlier into GLOBAL Q starvation (hips down ~6 orders), which collapsed P onto the measurement
   // floors and CAUSED the min-side S singularity. A cap that silently rescales the whole robot is worse than
   // the disease.
   private static final double QA_MAX = 900.0;
   // Reflected rotor inertia n^2 * J_rotor (kg*m^2) per joint, matched by case-insensitive joint-name substring
   // (Part B item 1). Added as a diagonal term to the Schur complement Lambda BEFORE inversion:
   // Lambda_eff = Lambda + diag(n_i^2 J_rotor,i). Physics: the rotor spins behind the gearbox about its own
   // axis and does not couple through the floating base, so the post-Schur diagonal add is simultaneously the
   // EXACT drivetrain term and a principled regularizer (it floors lambda_min(Lambda_eff) by Weyl). Distal
   // joints' link-side apparent inertia is as low as ~8e-4 kg*m^2 while their drivetrains reflect 0.05-0.07 —
   // without this term Lambda^-2 has diagonal outliers up to ~1.6e6 and Qa blows up for proximal/light joints.
   private static final String[] ROTOR_INERTIA_JOINT_KEYS = {
         "HIP_X", "HIP_Z", "HIP_Y", "KNEE", "ANKLE_Y", "ANKLE_X", "SPINE",
         "SHOULDER_Y", "SHOULDER_X", "SHOULDER_Z", "ELBOW",
         "WRIST_Z", "WRIST_X", "GRIPPER_Z", "NECK_Z", "NECK_Y"};
   private static final double[] ROTOR_INERTIA_VALUES = {
         0.062, 0.02, 0.167, 0.167, 0.07, 0.05, 0.062,
         0.067, 0.067, 0.022, 0.022,
         0.005, 0.005, 0.005, 0.005, 0.005};
   private static final double ROTOR_INERTIA_DEFAULT = 0.005; // conservative floor for an unmatched filtered joint
   // Gyro measurement-noise floor (Part B item 4). getAngularVelocityNoiseCovariance is zero on hardware whenever
   // the IMU's SensorNoiseParameters are unset (Alex historically ran with a null SensorNoiseParameters => all
   // covariances 0). A zero Sigma removes the innovation-covariance floor on the pure-bias rows of every 1-DoF
   // chain, collapsing lambda_min(S) (the logged 2.155e-12) and, via the Joseph K R K^T squaring loop, diverging
   // P. Any IMU whose gyro-noise trace is below SIGMA_GYRO_FLOOR_TRACE is floored to SIGMA_GYRO_FLOOR * I3, a
   // datasheet-plausible raw-gyro white-noise variance.
   private static final double SIGMA_GYRO_FLOOR = 1.0e-4;        // (0.01 rad/s)^2 per axis
   private static final double SIGMA_GYRO_FLOOR_TRACE = 3.0e-4;  // 3 * (0.01 rad/s)^2
   // Active innovation-covariance conditioning gate (Part B item 5). cond(S) is estimated from the Cholesky
   // factor diagonal ((max L_ii / min L_ii)^2 — no eigendecomposition); above this the whole stacked update is
   // skipped, because a finite-but-ill-conditioned S inverts to a huge gain that the Joseph K R K^T loop squares
   // each tick (the P divergence mechanism the LU/isFinite guards were blind to).
   private static final double COND_S_MAX = 1.0e9;
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
   // Separate one-shot flag for the near-singular innovation-covariance diagnostic (warnSingularInnovationOnce),
   // so its detailed S-conditioning report is not swallowed by an unrelated earlier non-finite-input warning.
   private boolean warnedSingularInnovation = false;

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
   // Conditioning telemetry (Part B items 2 & 5). Counters latch how often each safety path WOULD/DID engage
   // so a hardware run surfaces a model/config regression instead of hiding it; the gyro-block S diagnostics are
   // read straight off the Cholesky factor diagonal each tick (cheap, no eigendecomposition).
   private final YoInteger yoQaCapWouldBindCount = new YoInteger("jointKFQaCapWouldBindCount", registry);
   private final YoInteger yoInnovationGateSkipCount = new YoInteger("jointKFInnovationGateSkipCount", registry);
   private final YoDouble yoCondSProxyLog10 = new YoDouble("jointKFCondSProxyLog10", registry);
   private final YoDouble yoMinSDiag = new YoDouble("jointKFMinSDiag", registry);
   private final YoDouble yoMaxSDiag = new YoDouble("jointKFMaxSDiag", registry);

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
   private final DMatrixRMaj PHt = new DMatrixRMaj(0,0);
   private final DMatrixRMaj S = new DMatrixRMaj(0,0);
   private final DMatrixRMaj Sinv = new DMatrixRMaj(0,0);
   private final DMatrixRMaj K = new DMatrixRMaj(0,0);
   private final DMatrixRMaj nu = new DMatrixRMaj(0,0);
   private final DMatrixRMaj KR = new DMatrixRMaj(0,0); // I-KH, dim x dim, Joseph form
   private final DMatrixRMaj IKH = new DMatrixRMaj(0,0); // I-KH, dim x dim, Joseph form
   private final DMatrixRMaj Ptmp = new DMatrixRMaj(0,0);
   private final DMatrixRMaj Rimu = new DMatrixRMaj(3,3);
   private final DMatrixRMaj rot3 = new DMatrixRMaj(3,3);
   private final DMatrixRMaj anchorR = new DMatrixRMaj(3,3); // Sigma_eps, the stance-anchor model-error covariance

   // ---- Stacked gyro measurement (SPEC §5). ONE Joseph update per tick over z_g in R^{3(E+K)}: E = pairs
   // (rows [0, 3E)), K = active stance anchors this tick (rows [3E, 3E+3K)). This replaces Rev. 1's per-pair
   // sequential updates AND the separate phase-2 anchor. Biases and gyro white noise both enter through the
   // SAME rotational edge-incidence operator L, so a block-diagonal per-pair R is inconsistent (it double-
   // counts shared-IMU samples on a star topology, and both anchors' shared base sample in double support);
   // the stacked R_g = L Sigma L^T + Sigma_eps-block carries the exact cross-covariances (SPEC §5.3). The bias
   // columns of H_g ARE L: build L once per tick, use it in both places so the identity cannot drift.
   private int E;             // number of IMU pairs (fixed at construction)
   private int maxStackRows;  // 3 * (E + K_max), K_max = number of foot anchors; the widest stacked measurement
   private DMatrixRMaj Hg;    // 3(E+K) x dim measurement Jacobian [ 0 | J_stack(q^) | L(q^) ]
   private DMatrixRMaj zg;    // 3(E+K) x 1 stacked measurement (raw gyro samples)
   private DMatrixRMaj Rg;    // 3(E+K) x 3(E+K) stacked measurement noise L Sigma L^T + Sigma_eps
   private DMatrixRMaj Lmix;  // 3(E+K) x 3m mixing operator L (also copied into H_g's bias columns)
   private DMatrixRMaj Sigma; // 3m x 3m blkdiag of each IMU's angular-velocity MEASUREMENT-noise covariance
   private DMatrixRMaj LSigma; // 3(E+K) x 3m scratch for the L*Sigma product
   private IMUSensorReadOnly[] imusByOrdinal; // ordinal -> IMU, so the per-tick Sigma build is an index loop (no
                                              // map-iterator allocation on the estimator thread)
   private final DMatrixRMaj identity3 = new DMatrixRMaj(3,3); // constant I3 for anchor L-blocks
   // Previous tick's trusted-feet set: the phase-1 stacked update reads THIS tick's anchors from the set cached
   // by last tick's computeImuBiases (SPEC §6 phase note — the IHMC estimator finalizes contact trust in
   // phase 2, after computeJointState, so a phase-1 stacked anchor must use the prior tick's trust). Pre-sized
   // in allocate() and only cleared/refilled on the estimator thread (no per-tick allocation).
   private List<RigidBodyBasics> trustedFeetFromLastTick;
   // Reused LU solver for the innovation-covariance inverse, pre-sized in allocate(). This replaces the
   // per-tick CommonOps_DDRM.invert(S, Sinv), which allocates a fresh LU decomposition + solver on every
   // call once S is larger than 5x5 (i.e. the n-joint encoder update) — garbage on the estimator thread.
   // Cholesky (S = H P H^T + R is symmetric PD by construction). setA failure = non-PD => skip. Its factor L
   // (L L^T = S) also gives the conditioning/floor gate cheaply: S's pivots are L_ii^2, so cond(S) ~
   // (max L_ii / min L_ii)^2 with no eigendecomposition (Part B item 5).
   private LinearSolverChol_DDRM innovationSolver;
   private final DMatrixRMaj Lchol = new DMatrixRMaj(0, 0); // scratch for the innovation Cholesky factor L (k x k)
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
   // NOTE: CompositeRigidBodyMassMatrixCalculator is correct here. DynamicsMatrixCalculator merely wraps this
   // same calculator (it holds one internally, built over the whole-body-control toolbox) and computes the
   // identical M(q) — switching changes nothing numerically. Process-noise magnitude is governed by SIGMA_TAU
   // and the QA_MAX conditioning cap in updateProcessNoiseFromMassMatrix, not by the choice of calculator.
   private CompositeRigidBodyMassMatrixCalculator massMatrixCalculator;
   private int numNuisanceDoF;          // 6 (base) + number of gap joints; the marginalized block width N
   private int[] massMatrixNuisanceColumns; // nuisance DoF columns of the full mass matrix (length N)
   private int[] massMatrixColumn;      // filter joint state index -> DoF column in the calculator's mass matrix
   private final DMatrixRMaj MNN = new DMatrixRMaj(0, 0);       // nuisance block, N x N
   private final DMatrixRMaj MNf = new DMatrixRMaj(0, 0);       // nuisance-filtered coupling, N x n (= M_jN^T)
   private final DMatrixRMaj Mff = new DMatrixRMaj(0, 0);       // filtered block, n x n
   private final DMatrixRMaj MNNInvMNf = new DMatrixRMaj(0, 0); // X = M_NN^-1 M_Nf, N x n
   private final DMatrixRMaj Lambda = new DMatrixRMaj(0, 0);    // Schur complement Lambda_eff (rotor diag added in place), n x n
   private final DMatrixRMaj LambdaInv = new DMatrixRMaj(0, 0); // Lambda_eff^-1, n x n
   private final DMatrixRMaj Ytau = new DMatrixRMaj(0, 0);      // Lambda_eff^-1 with column j scaled by sigma_tau,j, n x n (Part B item 3)
   private final DMatrixRMaj Qa = new DMatrixRMaj(0, 0);        // Qa = Ytau Ytau^T (PSD, symmetric BY CONSTRUCTION), n x n
   private LinearSolverDense<DMatrixRMaj> nuisanceMassMatrixSolver; // Cholesky over M_NN, solves M_NN X = M_Nf
   private LinearSolverDense<DMatrixRMaj> schurSolver;          // Cholesky over Lambda_eff, inverts it
   // Reflected rotor inertia diagonals (Part B item 1), built at construction in filter/nuisance order.
   private double[] rotorInertiaDiag;          // length n, added to Lambda's diagonal (filter joint state order)
   private double[] nuisanceRotorInertiaDiag;  // length numNuisanceDoF: 0 on base rows, rotor inertia on gap-joint rows
   private double[] sigmaTauPerJoint;          // length n, sigma_tau,i = ALPHA * tau_max,i (Part B item 3)
   private boolean warnedMassMatrixFailure = false;
   private boolean warnedMassMatrixConditioningCap = false;
   private boolean sigmaFloorInitialized = false; // Sigma validated/floored/cached on first buildStackedMeasurement
   private boolean gyroSigmaFloored = false;   // any IMU's gyro noise was floored (Part B item 4)
   private boolean warnedGyroFloorRegression = false; // one-shot: R_g pair-row diagonal fell below the floor at runtime

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
         // Structural asserts (Part B item 6): a self-pair or a pair whose two IMUs sit on the same measurement
         // link has zero H, zero z and zero noise — S is singular by construction (SPEC §5). Fail loud at boot.
         if (parent == child)
            throw new IllegalArgumentException("JointLevelKFPreFilter: pair '" + pp.getEstimatorName() + "' has the same IMU ("
                  + parent.getSensorName() + ") as parent and child.");
         if (parent.getMeasurementLink() == child.getMeasurementLink())
            throw new IllegalArgumentException("JointLevelKFPreFilter: pair '" + pp.getEstimatorName() + "' parent and child IMUs "
                  + "share the measurement link " + parent.getMeasurementLink().getName() + " (zero-DoF chain).");
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

      // Acyclicity assert (Part B item 6): with the exact R_g, a cycle's telescoping row-combination has zero H,
      // zero z and zero noise, so S is singular BY CONSTRUCTION (SPEC §5.3). The used IMU graph MUST be a tree.
      assertAcyclicIMUGraph();

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
               if (!jointToIndex.containsKey(spanningJoint)) // unfiltered => gap joint => marginalize it
               {
                  nuisanceColumns.add(indexProvider.getJointDoFIndices(spanningJoint)[0]);
                  double gapRotor = lookupRotorInertia(spanningJoint.getName());
                  if (gapRotor < 0.0)
                     gapRotor = ROTOR_INERTIA_DEFAULT;
                  nuisanceRotor.add(gapRotor);
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
      // Per-joint reflected rotor inertia (item 1) and per-joint sigma_tau (item 3), in filter state order.
      // Built BEFORE allocate() because allocate() -> buildProcessNoise() -> updateProcessNoiseFromMassMatrix()
      // reads both.
      buildRotorInertiaAndSigmaTau();
      allocate();

      yoStateDimension.set(dim);
      yoNumberOfFilteredJoints.set(n);
      yoNumberOfIMUs.set(m);

      createJointYoVariables();

      // Boot-time census (Part B item 6): every hardware boot prints the chain-DoF layout and, for each 1-DoF
      // chain, the joint axis and its two pure-bias sentinel rows (the min-side rows whose S floor is Sigma).
      logChainDoFCensus();
   }

   /**
    * Union-find acyclicity check on the used IMU graph (Part B item 6). Each pair is an edge between its two
    * IMU ordinals; if an edge ever joins two already-connected IMUs the graph has a cycle, which makes the
    * stacked S singular by construction with the exact R_g (SPEC §5.3 — the cycle's telescoping row-combination
    * carries the identity 0 = 0: zero H, zero z, zero noise). THROW naming the redundant pair to drop (its
    * chain joints are already covered by the rest of the spanning tree, so dropping it loses no information).
    * Construction-only.
    */
   private void assertAcyclicIMUGraph()
   {
      int[] parent = new int[m];
      for (int i = 0; i < m; i++)
         parent[i] = i;
      for (Pair p : pairs)
      {
         int a = find(parent, imuToOrdinal.get(p.parent));
         int b = find(parent, imuToOrdinal.get(p.child));
         if (a == b)
            throw new IllegalStateException("JointLevelKFPreFilter: the used IMU graph has a CYCLE — the pair "
                  + p.parent.getSensorName() + "->" + p.child.getSensorName() + " closes a loop. With the exact R_g a "
                  + "cycle's telescoping row-combination has zero H, zero z and zero noise, so the stacked innovation "
                  + "covariance S is singular by construction (SPEC §5.3). Drop this redundant pair — its chain joints "
                  + "are already covered by the remaining edges, so removing it loses no information.");
         parent[a] = b;
      }
      // E <= m - 1 for a forest; with connectivity from the base IMU the used graph is a tree. (A disconnected
      // forest is not an error here — an IMU island simply contributes an independent sub-filter.)
   }

   private static int find(int[] parent, int i)
   {
      while (parent[i] != i)
      {
         parent[i] = parent[parent[i]];
         i = parent[i];
      }
      return i;
   }

   /** Reflected rotor inertia for a joint by case-insensitive name substring (Part B item 1); -1 if unmatched. */
   private static double lookupRotorInertia(String jointName)
   {
      String upper = jointName.toUpperCase();
      for (int i = 0; i < ROTOR_INERTIA_JOINT_KEYS.length; i++)
         if (upper.contains(ROTOR_INERTIA_JOINT_KEYS[i]))
            return ROTOR_INERTIA_VALUES[i];
      return -1.0;
   }

   /** Package-private test seam: the reflected rotor inertia the filter actually applies to the named joint —
    *  the table match, or the conservative default when unmatched. Lets the mass-matrix Qa oracle reconstruct
    *  Lambda_eff exactly (Part C), independent of whether a synthetic joint name happens to hit a table key. */
   static double reflectedRotorInertiaForNameOrDefault(String jointName)
   {
      double v = lookupRotorInertia(jointName);
      return v < 0.0 ? ROTOR_INERTIA_DEFAULT : v;
   }

   /**
    * Builds the per-joint reflected-rotor-inertia diagonal (Part B item 1) and the per-joint unmodeled-torque
    * STD sigma_tau,i = ALPHA * tau_max,i (Part B item 3), both in filter joint state order. Any joint with no
    * rotor-table match gets the conservative default floor (warn); any joint with no finite positive effort
    * limit falls back to the scalar SIGMA_TAU (warn). Construction-only; the per-tick process-noise update just
    * reads these arrays (allocation-free).
    */
   private void buildRotorInertiaAndSigmaTau()
   {
      rotorInertiaDiag = new double[n];
      sigmaTauPerJoint = new double[n];
      for (var e : jointToIndex.entrySet())
      {
         OneDoFJointBasics joint = e.getKey();
         int idx = e.getValue();

         double rotor = lookupRotorInertia(joint.getName());
         if (rotor < 0.0)
         {
            rotor = ROTOR_INERTIA_DEFAULT;
            LogTools.warn("JointLevelKFPreFilter: no reflected-rotor-inertia table entry for filtered joint '"
                          + joint.getName() + "'; applying the conservative default floor " + ROTOR_INERTIA_DEFAULT + " kg*m^2.");
         }
         rotorInertiaDiag[idx] = rotor;

         double tauMax = joint.getEffortLimitUpper();
         if (!Double.isFinite(tauMax) || tauMax <= 0.0)
         {
            LogTools.warn("JointLevelKFPreFilter: filtered joint '" + joint.getName() + "' has no finite positive effort"
                          + " limit (" + tauMax + "); falling back to the scalar SIGMA_TAU = " + SIGMA_TAU + " N*m for its sigma_tau.");
            sigmaTauPerJoint[idx] = SIGMA_TAU;
         }
         else
         {
            sigmaTauPerJoint[idx] = ALPHA * tauMax;
         }
      }
   }

   /**
    * Logs the chain-DoF census and, for each 1-DoF chain, the joint axis and its two pure-bias sentinel rows
    * (Part B item 6). The pure-bias rows are the two gyro axes orthogonal to the single joint axis: with a zero
    * gyro Sigma their innovation variance has no floor, so they are the min-side rows the Sigma floor (item 4)
    * protects. Construction-only string work; never on the hot path.
    */
   private void logChainDoFCensus()
   {
      StringBuilder sb = new StringBuilder("JointLevelKFPreFilter chain-DoF census (" + pairs.size() + " pairs over " + m + " IMUs, tree):");
      for (Pair p : pairs)
      {
         int dof = p.chainJoints.length;
         sb.append("\n  ").append(p.parent.getSensorName()).append("->").append(p.child.getSensorName())
           .append(" : ").append(dof).append("-DoF chain [").append(jointNamesOf(p.chainJoints)).append("]");
         if (dof == 1)
         {
            Vector3DReadOnly axis = p.chainJoints[0].getJointAxis();
            sb.append("  (1-DoF sentinel: joint axis ~[").append(String.format("%.2f,%.2f,%.2f", axis.getX(), axis.getY(), axis.getZ()))
              .append("]; the two rows orthogonal to it observe PURE bias — min-side S floor = Sigma)");
         }
      }
      LogTools.info(sb.toString());
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
      // Widest measurement is either the n-joint encoder block or the 3(E+K_max) stacked gyro block. Size all
      // the KF innovation scratch (PHt, S, Sinv, K, KR, nu) and the LU solver at this width so a full-anchor
      // stacked update never reallocates on the estimator thread.
      E = pairs.size();
      int kMax = footAnchors.size();
      maxStackRows = 3 * (E + kMax);
      int maxMeas = Math.max(n, maxStackRows);
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

      // Stacked gyro measurement scratch, pre-sized at the max (K = K_max active anchors); each tick reshapes
      // DOWN to the current 3(E+K) within this capacity, so no per-tick allocation. Lmix/Sigma/LSigma feed the
      // R_g = L Sigma L^T congruence and H_g's bias columns.
      Hg = new DMatrixRMaj(maxStackRows, dim);
      zg = new DMatrixRMaj(maxStackRows, 1);
      Rg = new DMatrixRMaj(maxStackRows, maxStackRows);
      Lmix = new DMatrixRMaj(maxStackRows, 3 * m);
      Sigma = new DMatrixRMaj(3 * m, 3 * m);
      LSigma = new DMatrixRMaj(maxStackRows, 3 * m);
      CommonOps_DDRM.setIdentity(identity3);
      trustedFeetFromLastTick = new ArrayList<>(Math.max(1, kMax));
      imusByOrdinal = new IMUSensorReadOnly[m];
      for (var e : imuToOrdinal.entrySet()) // construction-time only; the hot path indexes this array
         imusByOrdinal[e.getValue()] = e.getKey();

      // Sigma is validated + floored and cached on the FIRST buildStackedMeasurement (Part B item 4), not here:
      // an IMU's SensorNoiseParameters may be assigned after this filter is constructed (the package tests set
      // them post-construction; hardware sets them before the first tick), so deferring to first use captures the
      // real covariance instead of a construction-time zero. Once built it is reused every tick (never re-read).

      // Pre-size the Joseph-form scratch. IKH in particular MUST start at dim x dim: the first josephUpdate
      // does setIdentity(IKH) *before* multAdd reshapes it, so on a 0x0 IKH the identity is silently dropped
      // and the first covariance update becomes -KH instead of I-KH. Sizing both here also keeps their first
      // use from reallocating on the estimator thread.
      Ptmp.reshape(dim, dim);
      IKH.reshape(dim, dim);
      xBackup.reshape(dim, 1);
      PBackup.reshape(dim, dim);

      // Reused Cholesky solver for the innovation-covariance inverse (Part B item 5), warmed at the widest
      // measurement (maxMeas). S = H P H^T + R is symmetric PD by construction, so Cholesky is the right
      // factorization: setA returns false on a NON-PD S (a strictly stronger, better-conditioned reject than
      // LU's exact-singular test), and its factor L (L L^T = S) gives the conditioning/floor gate for free. The
      // warm-up forces every internal buffer to its largest size now, so per-tick setA()/invert() never allocate.
      innovationSolver = new LinearSolverChol_DDRM(new CholeskyDecompositionInner_DDRM(true)); // lower-triangular L
      innovationSolver.setA(CommonOps_DDRM.identity(maxMeas));
      Lchol.reshape(maxMeas, maxMeas);

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
      if (containsNonFinite(LambdaInv))
      {
         warnMassMatrixFailureOnce("non-finite Schur-complement inverse (near-singular Lambda_eff)");
         return;
      }

      // Per-joint Sigma_tau via the Gram form (Part B item 3): Y = Lambda_eff^-1 with column j scaled by
      // sigma_tau,j, then Qa = Y Y^T = Lambda_eff^-1 diag(sigma_tau^2) Lambda_eff^-T. Qa is PSD AND exactly
      // symmetric by construction, so the Van Loan fill below reads it directly (no symmetrized read). This
      // replaces the scalar Qa = sigma_tau^2 Lambda^-2; per-joint sigma_tau = ALPHA * tau_max,i scales the
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
         if (Qa.get(i, i) > maxQaDiag)
         {
            maxQaDiag = Qa.get(i, i);
            argMaxJoint = i;
         }
      }
      if (maxQaDiag > QA_MAX)
      {
         yoQaCapWouldBindCount.increment();
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
            + QA_MAX + ") at joint '" + jointNameByStateIndex(argMaxJointStateIndex) + "'. Qa is NOT rescaled (a uniform "
            + "scale would starve global Q, the old failure mode); this indicates a model/config regression — check the "
            + "reflected-rotor-inertia table entry and the mass matrix for this joint. Counting in jointKFQaCapWouldBindCount. Reported once.");
   }

   /**
    * Validates and floors each IMU's angular-velocity MEASUREMENT-noise covariance ONCE, at construction, and
    * fills the cached block-diagonal {@code Sigma} (Part B item 4). An IMU whose gyro-noise trace is zero /
    * non-finite (an unset SensorNoiseParameters — Alex historically ran with a null one, so every covariance was
    * the 3x3 zero) is an ERROR: its zero Sigma removes the innovation-covariance floor on the pure-bias rows of
    * every 1-DoF chain, which collapses lambda_min(S) and diverges P. An IMU whose trace is positive but below
    * the conditioning floor is a softer WARN. Either way the block is substituted with SIGMA_GYRO_FLOOR * I3. The
    * flooring decision is made HERE, once; the per-tick build copies this cached Sigma (never re-reads a
    * possibly-zero covariance).
    */
   private void buildAndFloorSigma()
   {
      Sigma.zero();
      for (int o = 0; o < m; o++)
      {
         imusByOrdinal[o].getAngularVelocityNoiseCovariance(Rimu);
         boolean nonFinite = containsNonFinite(Rimu);
         double trace = nonFinite ? Double.NaN : Rimu.get(0, 0) + Rimu.get(1, 1) + Rimu.get(2, 2);
         if (nonFinite || !(trace >= SIGMA_GYRO_FLOOR_TRACE))
         {
            gyroSigmaFloored = true;
            String name = imusByOrdinal[o].getSensorName();
            if (nonFinite || !(trace > 0.0))
               LogTools.error("JointLevelKFPreFilter: IMU '" + name + "' has an unset/zero/non-finite angular-velocity "
                     + "noise covariance (trace=" + trace + "). A zero Sigma removes the innovation-covariance floor on "
                     + "the pure-bias rows of every 1-DoF chain (the min-side S collapse). Substituting the gyro floor "
                     + SIGMA_GYRO_FLOOR + " (rad/s)^2 * I3; fix the sensor SensorNoiseParameters at the source.");
            else
               LogTools.warn("JointLevelKFPreFilter: IMU '" + name + "' gyro-noise trace " + trace + " (rad/s)^2 is below "
                     + "the conditioning floor " + SIGMA_GYRO_FLOOR_TRACE + "; substituting " + SIGMA_GYRO_FLOOR
                     + " * I3 for innovation-covariance conditioning.");
            Rimu.zero();
            for (int d = 0; d < 3; d++)
               Rimu.set(d, d, SIGMA_GYRO_FLOOR);
         }
         insertScaledInto(Rimu, 1.0, Sigma, 3 * o, 3 * o);
      }
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

      // Single stacked gyro update (SPEC §5-§6): all pair rows plus the active stance-anchor rows in ONE Joseph
      // update, using the previous tick's trusted-feet set (cached in computeImuBiases). The pairs+anchors share
      // IMU samples, so they MUST NOT be split — a per-block update cannot represent the shared-IMU noise
      // cross-covariance carried in R_g. If ANY entry of z_g/H_g/R_g is non-finite the WHOLE block is skipped
      // (never individual rows — a partial stack silently changes the bias-gauge structure, SPEC §6 step 3).
      buildStackedMeasurement();
      if (containsNonFinite(zg) || containsNonFinite(Hg) || containsNonFinite(Rg))
      {
         if (!warnedNonFiniteInput)
            warnNonFiniteInputOnce("stacked gyro measurement (pairs/anchors)");
      }
      else
      {
         josephUpdate(Hg, zg, Rg, "stackedGyroUpdate");
         warnIfNonFiniteState("stackedGyroUpdate", -1);
      }

      // P hygiene (Part B item 7): re-symmetrize P once per tick. The Joseph products keep P symmetric only to
      // round-off; a slow asymmetry drift eventually corrupts the Cholesky conditioning gate. O(dim^2),
      // negligible next to the Joseph O(dim^3) products.
      symmetrize(P);

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

   /**
    * Assembles the stacked gyro measurement (H_g, z_g, R_g, and the mixing operator L = {@code Lmix}) for this
    * tick, WITHOUT applying the update (SPEC §5). Rows: pair {@code e} occupies rows [3e, 3e+3); the active
    * stance anchors (feet trusted last tick) follow at rows [3E, 3E+3K). Frames are already current — the
    * estimator calls {@code updateFramesRecursively()} every tick — so, as in the Rev. 1 pair build, we do NOT
    * re-update them here (redundant and allocates inside MovingReferenceFrame.update()). Allocation-free: every
    * matrix is pre-sized in {@link #allocate()} and only reshaped DOWN here.
    *
    * <p>Convention-bound lines (SPEC §9, "convention traps") — each fails silently if wrong, so all are called
    * out here and pinned by the nuisance-marginalization oracle test:</p>
    * <ul>
    *   <li>pair residual z_e = R(child-&gt;J_e) w_child - R(parent-&gt;J_e) w_parent (child +, parent -),
    *   matching {@code setKinematicChain(parent, child)};</li>
    *   <li>L pair blocks: +R(child-&gt;J_e) in the child IMU's bias column, -R(parent-&gt;J_e) in the parent's;</li>
    *   <li>anchor: H q̇-columns -J_leg, L block +I3 in the base IMU's column (J frame = base measurement frame,
    *   so the anchor's own rotation block is exactly identity), z = raw base gyro;</li>
    *   <li>R_g = L Sigma L^T + Sigma_eps on the anchor diagonals, Sigma = each IMU's angular-velocity
    *   MEASUREMENT-noise covariance (NOT the bias process-noise covariance).</li>
    * </ul>
    */
   private void buildStackedMeasurement()
   {
      // Validate + floor + cache Sigma on first use (Part B item 4). One-time; the warn/error strings build only
      // on this first fire. After this the cached Sigma is reused unchanged (never re-read from the IMUs).
      if (!sigmaFloorInitialized)
      {
         buildAndFloorSigma();
         sigmaFloorInitialized = true;
      }

      // Mark active anchors (usable + trusted last tick, SPEC §6 phase note) and count them.
      int activeAnchors = 0;
      for (int i = 0; i < footAnchors.size(); i++)
      {
         FootAnchor fa = footAnchors.get(i);
         fa.active = fa.usable && trustedFeetFromLastTick.contains(fa.foot);
         if (fa.active)
            activeAnchors++;
      }
      int rows = 3 * (E + activeAnchors);

      Hg.reshape(rows, dim);       Hg.zero();
      zg.reshape(rows, 1);         zg.zero();
      Lmix.reshape(rows, 3 * m);   Lmix.zero();
      Rg.reshape(rows, rows);      Rg.zero();

      // Sigma (blkdiag over IMUs of each IMU's angular-velocity MEASUREMENT-noise covariance, in its own frame)
      // is a measurement-noise CONSTANT, built and FLOORED once at construction (buildAndFloorSigma, Part B
      // item 4). It is NOT rebuilt here — re-reading getAngularVelocityNoiseCovariance every tick would re-admit
      // a possibly-zero covariance (the min-side S collapse). Deliberately NOT the bias process-noise covariance
      // (the bias random-walk intensity, orders of magnitude smaller — a Rev. 1 over-confidence bug class).

      // Pair rows.
      for (int e = 0; e < E; e++)
      {
         Pair p = pairs.get(e);
         int row0 = 3 * e;
         p.jac.reset();
         CommonOps_DDRM.extract(p.jac.getJacobianMatrix(), 0, 3, 0, p.qdCols.length, p.Jang, 0, 0); // angular part

         // z_e = omega_child - omega_parent expressed in the child body-fixed Jacobian frame (child +, parent -).
         fvA.setToZero(p.child.getMeasurementFrame());
         fvA.set(p.child.getAngularVelocityMeasurement());
         fvA.changeFrame(p.jac.getJacobianFrame());
         fvB.setToZero(p.parent.getMeasurementFrame());
         fvB.set(p.parent.getAngularVelocityMeasurement());
         fvB.changeFrame(p.jac.getJacobianFrame());
         fvA.sub(fvB);
         zg.set(row0, 0, fvA.getX());
         zg.set(row0 + 1, 0, fvA.getY());
         zg.set(row0 + 2, 0, fvA.getZ());

         // H_g q̇-columns: +J_e scattered onto the path joints. q-columns left 0 (SPEC §5.4); bias columns via L.
         for (int c = 0; c < p.qdCols.length; c++)
            for (int r = 0; r < 3; r++)
               Hg.set(row0 + r, p.qdCols[c], p.Jang.get(r, c));

         // L pair blocks: +R(child->J_e) at the child IMU bias column, -R(parent->J_e) at the parent IMU column.
         // Lmix column = state bias column minus the 2n (q,q̇) offset (p.childBias/parentBias were precomputed).
         packRotationToJacFrame(p.child, p.jac.getJacobianFrame(), rot3);
         insertScaledInto(rot3, +1.0, Lmix, row0, p.childBias - 2 * n);
         packRotationToJacFrame(p.parent, p.jac.getJacobianFrame(), rot3);
         insertScaledInto(rot3, -1.0, Lmix, row0, p.parentBias - 2 * n);
      }

      // Active stance-anchor rows (SPEC §5.2): z = raw base gyro; H q̇-columns -J_leg; L block +I3 on the base IMU.
      int arow = 3 * E;
      for (int i = 0; i < footAnchors.size(); i++)
      {
         FootAnchor fa = footAnchors.get(i);
         if (!fa.active)
            continue;
         fa.jac.reset();
         CommonOps_DDRM.extract(fa.jac.getJacobianMatrix(), 0, 3, 0, fa.qdCols.length, fa.Jang, 0, 0);

         Vector3DReadOnly w = baseIMU.getAngularVelocityMeasurement();
         zg.set(arow, 0, w.getX());
         zg.set(arow + 1, 0, w.getY());
         zg.set(arow + 2, 0, w.getZ());

         for (int c = 0; c < fa.qdCols.length; c++)
            for (int r = 0; r < 3; r++)
               Hg.set(arow + r, fa.qdCols[c], -fa.Jang.get(r, c)); // -J_leg (moving omega_foot to the LHS)

         insertScaledInto(identity3, +1.0, Lmix, arow, baseBiasCol - 2 * n); // +I3, base measurement frame
         arow += 3;
      }

      // Copy L into H_g's bias columns [2n, 2n+3m): "the bias columns of H_g ARE L" (SPEC §5.3), one build, two uses.
      for (int r = 0; r < rows; r++)
         for (int c = 0; c < 3 * m; c++)
            Hg.set(r, 2 * n + c, Lmix.get(r, c));

      // R_g = L Sigma L^T + Sigma_eps-block. The congruence reproduces the SPEC §5.3 block table exactly (pair
      // diagonals, shared-IMU s_e*s_f cross-blocks, pair×anchor, anchor×anchor Sigma_base). Allocation-free:
      // LSigma/Rg are pre-sized. Then add Sigma_eps (anchorR) on each active anchor's 3x3 diagonal.
      LSigma.reshape(rows, 3 * m);
      CommonOps_DDRM.mult(Lmix, Sigma, LSigma);
      CommonOps_DDRM.multTransB(LSigma, Lmix, Rg);
      int arow2 = 3 * E;
      for (int i = 0; i < footAnchors.size(); i++)
      {
         if (!footAnchors.get(i).active)
            continue;
         for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
               Rg.add(arow2 + r, arow2 + c, anchorR.get(r, c));
         arow2 += 3;
      }
      // Exact symmetry for the Joseph update (the congruence is symmetric only to round-off). Reuses the
      // Schur path's helper.
      symmetrize(Rg);

      // Runtime floor tripwire (Part B item 4). Each pair row of R_g = L Sigma L^T is a rotation-congruence of
      // the floored Sigma blocks (parent + child), so its diagonal is ~2 * floor and cannot fall below the floor
      // unless the L/Sigma assembly regresses. Warn once if any pair-row diagonal drops below half the floor.
      if (!warnedGyroFloorRegression)
      {
         double minPairDiag = Double.POSITIVE_INFINITY;
         for (int r = 0; r < 3 * E; r++)
            minPairDiag = Math.min(minPairDiag, Rg.get(r, r));
         if (minPairDiag < 0.5 * SIGMA_GYRO_FLOOR)
         {
            warnedGyroFloorRegression = true;
            LogTools.warn("JointLevelKFPreFilter: min R_g pair-row diagonal " + minPairDiag + " fell below half the gyro floor "
                  + SIGMA_GYRO_FLOOR + " — the Sigma floor should make this impossible; the stacked-measurement assembly (L or "
                  + "Sigma) has regressed. Reported once.");
         }
      }
   }

   // ================================ Phase 2 ================================

   @Override
   public void computeImuBiases(List<RigidBodyBasics> trustedFeet)
   {
      // Phase 2 is now bookkeeping only (SPEC §6 phase note): the stance anchors have moved into the phase-1
      // stacked update, so all this does is cache THIS tick's trusted-feet set for next tick's anchor rows.
      // Splitting the anchors back into a phase-2 update would make the base-gyro correlation between the
      // anchors and the base-adjacent pairs unexpressible (sequential updates cannot carry cross-block noise
      // correlation) — which is exactly the bug this revision removes, so do NOT run a measurement update here.
      // Cleared + refilled with an index loop (not addAll, which allocates via Collection.toArray); the list is
      // pre-sized to the anchor count in allocate(), so this stays allocation-free on the estimator thread.
      trustedFeetFromLastTick.clear();
      if (trustedFeet != null)
      {
         for (int i = 0; i < trustedFeet.size(); i++)
            trustedFeetFromLastTick.add(trustedFeet.get(i));
      }
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
      // Cholesky factor + ACTIVE conditioning/floor gate (Part B item 5). symmetrize first (Cholesky assumes
      // exact symmetry; S is symmetric only to round-off). setA returns false on a NON-PD S => skip. Then read
      // the factor diagonal: S's pivots are L_ii^2, so cond(S) ~ (max L_ii / min L_ii)^2 and lambda_min(S) ~
      // (min L_ii)^2 — both with NO eigendecomposition. A finite-but-ill-conditioned S (cond > COND_S_MAX)
      // inverts to a huge gain the Joseph K R K^T loop squares each tick — the divergence the old LU/isFinite
      // guards were blind to — so the WHOLE block is skipped. The floor gate catches the dual pathology: a pivot
      // below half the applicable measurement-noise floor (ENCODER_VAR / SIGMA_GYRO_FLOOR) is algebraically
      // impossible for a healthy S = H P H^T + R and signals a collapsed (zero-Sigma) row.
      symmetrize(S);
      if (!innovationSolver.setA(S))
      {
         warnSingularInnovationOnce(label, "S is not positive definite (Cholesky factorization rejected it)", Hm, Rm);
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
      if (label != null && label.startsWith("stackedGyro")) // publish the gyro-block S diagnostics every tick (cheap)
      {
         yoCondSProxyLog10.set(Math.log10(condProxy));
         yoMinSDiag.set(minPivot);
         yoMaxSDiag.set(maxPivot);
      }
      if (condProxy > COND_S_MAX)
      {
         yoInnovationGateSkipCount.increment();
         warnSingularInnovationOnce(label, String.format("cond(S) proxy %.3e exceeds COND_S_MAX %.1e; whole block gated", condProxy, COND_S_MAX), Hm, Rm);
         return;
      }
      double rFloor = (label != null && label.startsWith("encoder")) ? ENCODER_VAR : SIGMA_GYRO_FLOOR;
      if (minPivot < 0.5 * rFloor)
      {
         yoInnovationGateSkipCount.increment();
         warnSingularInnovationOnce(label, String.format("min S pivot %.3e below half the R floor %.3e (collapsed row); block gated", minPivot, rFloor), Hm, Rm);
         return;
      }
      innovationSolver.invert(Sinv);
      // A PD, well-conditioned S still gets a residual-overflow guard before x/P are touched.
      if (containsNonFinite(Sinv))
      {
         warnSingularInnovationOnce(label, "S^-1 is non-finite (S is near-singular and its inverse overflowed)", Hm, Rm);
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

   /**
    * One-shot diagnostic for a singular / near-singular innovation covariance S = H P H^T + R, which forces the
    * offending measurement update to be skipped — a hidden way the filter keeps diverging even after the Qa cap
    * binds (a near-singular Lambda inflates P, which makes S ill-conditioned). Names WHICH physical measurement
    * is degenerate: S is symmetric PSD, so the eigenvector of its smallest eigenvalue is the measurement
    * direction with almost no innovation variance; the rows carrying the most weight in it are the pairs / stance
    * anchors / encoder joints driving the ill-conditioning. The eigen/string work runs only on this single
    * occurrence — never on the healthy hot path.
    *
    * @param label  the update ("encoder" / "stackedGyroUpdate"); selects the row-to-physical-element mapping.
    * @param reason how S was found bad (exactly singular vs inverse overflowed).
    * @param Hm     this update's measurement Jacobian. S is recomputed from it because the solver's {@code setA}
    *               may have overwritten the shared S buffer in place with its LU factors.
    * @param Rm     this update's measurement noise.
    */
   private void warnSingularInnovationOnce(String label, String reason, DMatrixRMaj Hm, DMatrixRMaj Rm)
   {
      if (warnedSingularInnovation)
         return;
      warnedSingularInnovation = true;
      LogTools.warn(describeSingularInnovation(label, reason, Hm, Rm) + " Reported once.");
   }

   /**
    * Builds the singular-innovation diagnostic string used by {@link #warnSingularInnovationOnce}. Package-private
    * so a test can assert the row -> physical-element mapping directly instead of scraping the log output.
    */
   String describeSingularInnovation(String label, String reason, DMatrixRMaj Hm, DMatrixRMaj Rm)
   {
      int k = Hm.getNumRows();
      // Recompute S = H P H^T + R fresh (setA above may have decomposed the field S in place). Rare path — ok to allocate.
      DMatrixRMaj phT = new DMatrixRMaj(dim, k);
      DMatrixRMaj Sfresh = new DMatrixRMaj(k, k);
      CommonOps_DDRM.multTransB(P, Hm, phT);
      CommonOps_DDRM.mult(Hm, phT, Sfresh);
      CommonOps_DDRM.addEquals(Sfresh, Rm);

      StringBuilder msg = new StringBuilder();
      msg.append("JointLevelKFPreFilter: near-singular innovation covariance in the ").append(label)
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
         // Report BOTH ends (Part B item 5). When the pathology is a lambda_MAX blow-up (P diverging), the
         // SMALLEST eigenvector points at the HEALTHIEST rows — so naming only the min side (as the prior
         // diagnostic did) mis-blames the stance-leg encoders. Compare lambda_max against a sane innovation
         // scale to say which side is actually pathological.
         final double SANE_INNOVATION_SCALE = 1.0e6;
         boolean maxSidePathological = maxEig > SANE_INNOVATION_SCALE;
         msg.append(String.format("cond(S)=%.3e, lambda_min=%.3e, lambda_max=%.3e — the %s side is pathological. ",
                                  cond, minEig, maxEig,
                                  maxSidePathological ? "MAX (lambda_max above a sane innovation scale ~1e6; P has diverged)"
                                                      : "MIN (lambda_min collapsed toward 0; a row lost its noise floor)"));
         msg.append("min-side rows (near-null measurement directions): ");
         appendDominantRows(msg, eig.getEigenVector(minIdx), k, label, Hm);
         msg.append(". max-side rows (P-blowup directions): ");
         appendDominantRows(msg, eig.getEigenVector(maxIdx), k, label, Hm);
      }
      else
      {
         int minRow = 0;
         double minDiag = Double.POSITIVE_INFINITY;
         for (int i = 0; i < k; i++)
            if (Sfresh.get(i, i) < minDiag) { minDiag = Sfresh.get(i, i); minRow = i; }
         msg.append(String.format("(eigen-decomposition failed; smallest S diagonal %.3e at %s)",
                                  minDiag, describeMeasurementRow(minRow, label, Hm)));
      }
      msg.append(" This is where the innovation ill-conditioning lives.");
      return msg.toString();
   }

   /** Appends the up-to-4 rows carrying the most weight in eigenvector {@code v}, as "|weight| -> physical
    *  element", for the both-ends singular-innovation diagnostic. Diagnostic-only; never on the hot path. */
   private void appendDominantRows(StringBuilder msg, DMatrixRMaj v, int k, String label, DMatrixRMaj Hm)
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
         msg.append(String.format("%.2f -> %s", bestAbs, describeMeasurementRow(best, label, Hm)));
         reported++;
      }
      if (reported == 0)
         msg.append("(spread across the whole block; no single row dominates)");
   }

   /**
    * Maps a measurement row to the physical element it observes, for the singular-innovation diagnostic. Encoder
    * rows are mapped through {@code Hm} (the observed joint is the column with the unit entry), so this stays
    * correct even if the encoder update is later gated to a joint subset. Stacked gyro rows follow the SPEC §5
    * layout: rows [3e,3e+3) are pair e; rows [3E,...) are the active stance anchors in {@code footAnchors} order.
    * Diagnostic-only; never on the hot path.
    */
   private String describeMeasurementRow(int row, String label, DMatrixRMaj Hm)
   {
      if (label != null && label.startsWith("encoder"))
      {
         int col = -1;
         for (int c = 0; c < Hm.getNumCols(); c++)
            if (Hm.get(row, c) != 0.0) { col = c; break; }
         return "encoder q of joint " + ((col >= 0 && col < n) ? jointNameByStateIndex(col) : "?");
      }
      int block = row / 3;
      int axis = row % 3;
      char ax = axis == 0 ? 'x' : axis == 1 ? 'y' : 'z';
      if (block < E)
      {
         Pair p = pairs.get(block);
         return "gyro pair " + block + " (" + p.parent.getSensorName() + "->" + p.child.getSensorName()
                + "), axis " + ax + ", chain=[" + jointNamesOf(p.chainJoints) + "]";
      }
      FootAnchor fa = nthActiveAnchor(block - E);
      if (fa != null)
         return "stance anchor (foot " + fa.foot.getName() + "), axis " + ax + ", leg=[" + jointNamesOf(fa.legJoints) + "]";
      return "gyro row " + row + " (unmapped)";
   }

   /** Reverse of {@code jointToIndex} for the diagnostic path: filter-joint name at a given state index. */
   private String jointNameByStateIndex(int stateIndex)
   {
      for (var e : jointToIndex.entrySet())
         if (e.getValue() == stateIndex)
            return e.getKey().getName();
      return "?idx" + stateIndex;
   }

   /** The {@code activeIdx}-th anchor with {@code active == true}, in footAnchors order (matches the stacked-row layout). */
   private FootAnchor nthActiveAnchor(int activeIdx)
   {
      int count = 0;
      for (int i = 0; i < footAnchors.size(); i++)
      {
         FootAnchor fa = footAnchors.get(i);
         if (fa.active)
         {
            if (count == activeIdx)
               return fa;
            count++;
         }
      }
      return null;
   }

   private static String jointNamesOf(OneDoFJointBasics[] joints)
   {
      StringBuilder sb = new StringBuilder();
      for (int i = 0; i < joints.length; i++)
      {
         if (i > 0)
            sb.append(",");
         sb.append(joints[i].getName());
      }
      return sb.toString();
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

   /**
    * Builds the stacked gyro measurement (H_g, z_g, R_g, L) from the current model configuration and the cached
    * trusted-feet-from-last-tick set, WITHOUT applying the update, for inspection. Set the active anchors first
    * with {@link #setTrustedFeetForTest} (empty => pairs-only, K = 0). Replaces the Rev. 1 per-pair
    * {@code buildPairMeasurementForTest} seam — the filter no longer has a per-pair update path.
    */
   void buildStackedMeasurementForTest() { buildStackedMeasurement(); }
   DMatrixRMaj getStackedMeasurementJacobian() { return Hg.copy(); }  // H_g (3(E+K) x dim), reshaped to this tick
   DMatrixRMaj getStackedMeasurementResidual() { return zg.copy(); }  // z_g (3(E+K) x 1)
   DMatrixRMaj getStackedMeasurementNoise()    { return Rg.copy(); }  // R_g (3(E+K) x 3(E+K))
   DMatrixRMaj getMixingOperator()             { return Lmix.copy(); } // L (3(E+K) x 3m); H_g bias columns == this
   int getStackedRowForPair(int pairIndex)     { return 3 * pairIndex; }      // pair e occupies rows [3e, 3e+3)
   int getBiasBlockColumn(IMUSensorReadOnly imu) { return 2 * n + 3 * imuToOrdinal.get(imu); } // state col of imu's bias
   int getImuOrdinal(IMUSensorReadOnly imu)    { return imuToOrdinal.get(imu); }
   IMUSensorReadOnly getBaseIMU()              { return baseIMU; }

   /** Overwrites the cached previous-tick trusted-feet set that {@link #buildStackedMeasurement} reads anchors from. */
   void setTrustedFeetForTest(List<RigidBodyBasics> feet)
   {
      trustedFeetFromLastTick.clear();
      if (feet != null)
         for (int i = 0; i < feet.size(); i++)
            trustedFeetFromLastTick.add(feet.get(i));
   }
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
      boolean active; // true this tick: usable AND trusted last tick (SPEC §6); set in buildStackedMeasurement
      final DMatrixRMaj Jang = new DMatrixRMaj(3,1);
      FootAnchor(RigidBodyBasics foot)
      {
         this.foot = foot;
      }
   }

}

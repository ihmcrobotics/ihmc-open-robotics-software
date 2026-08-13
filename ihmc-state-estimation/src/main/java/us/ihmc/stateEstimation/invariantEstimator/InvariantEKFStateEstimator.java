package us.ihmc.stateEstimation.invariantEstimator;

import java.util.Objects;

import gnu.trove.map.TObjectDoubleMap;
import org.apache.commons.math3.distribution.ChiSquaredDistribution;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;
import us.ihmc.stateEstimation.jointLevel.ZeroIMUBiasProvider;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Adapter that runs the {@link InvariantEKF} as an IHMC {@link StateEstimatorController}.
 *
 * <p>Intended as a <em>secondary</em> (evaluation) estimator: it runs alongside the main estimator,
 * reads the same sensors, produces its own floating-base estimate, and exposes it as YoVariables for
 * comparison. It does not write to the shared robot model and does not drive the controller.</p>
 *
 * <p>Per tick it (1) reads the IMU ω and a and expresses them in the pelvis (base) frame, predicts;
 * then (2) for each foot it forms the body-frame contact measurement y = sole-origin-in-pelvis-frame
 * (= Rᵀ(p_Cᵢ − p_B)) and applies a contact update. For this first standing version both feet are
 * treated as always in contact — no touchdown/lift-off bookkeeping.</p>
 *
 * <p>Forward kinematics use the shared estimator robot model, whose joints are updated upstream each
 * tick (the main estimator runs first); base-relative quantities like the sole-in-pelvis position are
 * independent of the base pose, so they are valid regardless of which estimator owns the root.</p>
 */
public class InvariantEKFStateEstimator implements StateEstimatorController
{
   private static final int NUMBER_OF_CONTACTS = 2; // left, right feet
   private static final SideDependentList<Integer> CONTACT_INDICES = new SideDependentList<>(0, 1);

   // Filter-consistency check: each contact update is a 3-DOF position measurement, so its NIS is
   // χ²-distributed with 3 DOF. The two-sided CONSISTENCY_CONFIDENCE band gives the acceptance interval
   // NIS should stay inside (~CONSISTENCY_CONFIDENCE of the time) when the filter is well-tuned.
   private static final int CONTACT_MEASUREMENT_DOF = 3;
   private static final double CONSISTENCY_CONFIDENCE = 0.95;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final InvariantEKF ekf;
   private final double dt;
   private final double initialCovariance;

   private final HumanoidReferenceFrames referenceFrames;
   private final MovingReferenceFrame pelvisFrame;
   private final SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();

   private final SensorOutputMapReadOnly sensorOutputMap;
   private final IMUSensorReadOnly imuSensor;
   private final IMUBiasProvider imuBiasProvider;

   private ContactMeasurementNoiseProvider contactMeasurementNoiseProvider;

   // Per-tick temporaries.
   private final FrameVector3D angularVelocity = new FrameVector3D();
   /** RAW gyro in the pelvis frame (no bias correction). The quasi-static gate must not see a runaway bias. */
   private final FrameVector3D rawAngularVelocity = new FrameVector3D();
   private final FrameVector3D linearAcceleration = new FrameVector3D();

   // Safeguard on the gyro bias handed up by the joint-level pre-filter. That bias is subtracted from the raw
   // gyro and integrated straight into base orientation, so a runaway upstream estimate (observed ~0.45 rad/s on
   // hardware, 2026-07-10) drives the base pitch to drift. A physical MEMS gyro bias is ~0.001-0.01 rad/s; clamp
   // per axis so a bad upstream bias cannot destroy the estimate, and PUBLISH the applied bias (previously an
   // unlogged black box that had to be backed out arithmetically from the drift).
   // 0.02 rad/s = 1.1 deg/s. The previous 0.05 was not a safeguard: it licensed 2.9 deg/s (172 deg/min) of
   // open-loop drift, and in log 20260712_163634 the applied bias sat PINNED at +-0.05 on two of three axes for
   // minutes (JointKF truth: 0.152/-0.021/0.072). A saturating clamp is a tripwire that has already fired, not a
   // fix -- the fix is the stance-anchor gauge restoration in JointLevelKFPreFilter (FINDINGS.md Part F). With
   // that in place a healthy bias is O(0.001-0.01), so invariantGyroBiasClampCount should now stay ~0; a rising
   // count means the gauge is unfixed again.
   private static final double MAX_GYRO_BIAS = 0.02; // rad/s per axis
   private final FrameVector3D appliedGyroBias = new FrameVector3D();
   /**
    * The same applied bias, re-expressed in the pelvis frame for logging only. The pelvis IMU is mounted yawed
    * +90 deg about Z (PELVIS_IMU_JOINT rpy="0.005114 -0.0001262 1.570796"), so an IMU-frame bias axis is NOT the
    * pelvis axis of the same name: pelvis_X(roll) = -imu_Y and pelvis_Y(pitch) = +imu_X. Publishing the bias only
    * in the IMU frame next to pelvis-frame rates invites reading a pitch bias as a roll bias. Kept separate from
    * {@link #appliedGyroBias}, which MUST stay in the IMU frame for the frame-checked sub() below.
    */
   private final FrameVector3D appliedGyroBiasInPelvis = new FrameVector3D();
   private final FramePoint3D contactInBody = new FramePoint3D();
   private final FramePoint3D contactInWorld = new FramePoint3D();
   private final RotationMatrix tempRotation = new RotationMatrix();
   private final Vector3D tempVector = new Vector3D();
   private final FramePose3D mainEstimatePelvisPose = new FramePose3D();
   private final FrameVector3D mainLinearVelocity = new FrameVector3D();           // DRC root linear velocity → world
   private final Vector3D invariantLinearVelocityWorld = new Vector3D();           // invariant base velocity (world)
   private final Vector3D mainAngularVelocityBody = new Vector3D();                // DRC root angular velocity (body)
   private final Vector3D invariantAngularVelocityBody = new Vector3D();           // gyro the filter integrates (body)
   private final Vector3D velocityErrorTemp = new Vector3D();

   /**
    * True when this filter has been promoted to main and drives the shared robot model's root joint (see
    * {@link InvariantMainStateEstimator}). In that mode the shared pelvis frame is this filter's own
    * previous-tick output, so the "invariantMinusMain*" comparisons would be stale self-comparisons; they
    * are set to NaN instead so they cannot be misread as a DRC comparison.
    */
   private boolean runningAsMain = false;
   /** Volatile: written from the controller thread via {@link #requestStateEstimatorMode}, read in doControl(). */
   private volatile StateEstimatorMode operatingMode = StateEstimatorMode.NORMAL;
   private boolean heldLastTick = false;
   private static final double CONTACT_HOLD_THRESHOLD = 0.5; // both feet below -> base translation will be unobservable
   private final Vector3D zeroVelocity = new Vector3D(); // final, stays zero

   // Gravity-leveling (tilt) update: the accelerometer-as-gravity-reference measurement that gives roll/pitch a
   // DIRECT observation (the contact update's Jacobian has a zero rotation block, so without this pitch/roll only
   // drift open-loop off the gyro). Applied only when quasi-static so real linear acceleration isn't mistaken for
   // tilt; the tilt-error DIAGNOSTIC is computed every tick regardless. TODO(retune) the gate/variance vs NIS.
   private boolean gravityLevelingEnabled = true;
   private static final double QUASI_STATIC_ACCEL_TOLERANCE = 0.05; // ‖a‖ within ±5% of |g|
   private static final double QUASI_STATIC_GYRO_THRESHOLD = 0.15;  // rad/s; "barely rotating"
   private static final double QUASI_STATIC_HORIZONTAL_ACCEL_THRESHOLD = 0.5; // m/s²; horizontal specific force (≈2.9° tilt at |g|)

   // Optional: restrict the PITCH leveling to double support (roll is always leveled). Fore-aft accel — which
   // fakes pitch tilt and leans the base forward — is smallest with both feet planted. Default OFF: the
   // anisotropic σ_pitch + horizontal-accel gate are the primary defense, and keeping pitch observable in single
   // support leaves the tilt diagnostic live to root-cause any residual pitch divergence (rather than hide it).
   private boolean gatePitchOnDoubleSupport = false;
   private static final double DOUBLE_SUPPORT_CONTACT_PROBABILITY = 0.5; // per-foot p above which a foot counts as planted

   // Soft contact handling: a per-foot contact probability p ∈ [0,1] drives two covariance knobs each tick.
   // The default provider is forward-kinematics only (runs on hardware); ContactNet replaces it later.
   private ContactProbabilityProvider contactProbabilityProvider;
   private final double baseContactVariance;                  // σ_c² at full contact (p = 1)
   private double swingMeasurementInflation = 9.0e1;          // R_i  ×= inflation^(1−p): muted swing foot at p → 0
   private double swingSlipInflation = 9.0e1;                 // σ_{c,i}² ×= inflation^(1−p): forgetful anchor at p → 0
   private final Matrix3D inflatedContactCovariance = new Matrix3D();
   private final SideDependentList<YoDouble> yoContactProbability;

   // Filter-consistency: per-foot Normalized Innovation Squared (NIS = rᵀS⁻¹r) from the contact update,
   // plus the shared two-sided χ² acceptance band it should stay inside (same DOF for both feet, so one
   // pair). Set NaN on ticks where the contact update is skipped (frozen / no contact).
   private final SideDependentList<YoDouble> yoContactNIS;
   private final YoDouble yoContactNISLowerBound = new YoDouble("invariantContactNISLowerBound", registry);
   private final YoDouble yoContactNISUpperBound = new YoDouble("invariantContactNISUpperBound", registry);

   // S-decomposition diagnostics (2026-07-16, NIS-miscalibration investigation): S = H·P·Hᵀ + R per contact
   // update. The constant-R sweeps moved NIS only ~3x at R/70 (replay A/B), so these attribute the S trace to
   // its three inputs: the state-covariance share (HPHᵀ), the applied measurement noise AFTER the swing-foot
   // probability inflation (R), and the inflation factor itself. P-block traces attribute HPHᵀ further:
   // trace(HPHᵀ) = tr(P_base) + tr(P_contact) − 2·tr(P_cross) for this H (±I blocks).
   private final SideDependentList<YoDouble> yoContactSHPHtTrace;
   private final SideDependentList<YoDouble> yoContactSRTrace;
   private final SideDependentList<YoDouble> yoContactRInflation;
   private final SideDependentList<YoDouble> yoContactPContactTrace;
   private final YoDouble yoContactPBaseTrace = new YoDouble("invariantContactPBaseTrace", registry);

   // H4 (anchor-switch transient) diagnostics: the applied state correction K·r per contact update,
   // split by tangent block. If intra-stance error is released as a pulse when the anchor set
   // changes, these spike at jointKFActiveAnchorCount transitions relative to mid-stance.
   private final SideDependentList<YoDouble> yoContactCorrectionRotNorm;
   private final SideDependentList<YoDouble> yoContactCorrectionVelNorm;
   private final SideDependentList<YoDouble> yoContactCorrectionPosNorm;

   // H4 Phase 2: touchdown anchor re-seed (see InvariantEKF.reseedContact and the 2026-07-16
   // derivation note). Kill switch defaults ON for replay evaluation; one-shot-per-cycle hysteresis
   // guards edge chatter: re-seed fires once on the rising probability crossing and re-arms only
   // after probability stays below the (lower) re-arm threshold for a sustained dwell — a chattering
   // edge cannot re-zero the residual repeatedly (which would silently degenerate into DRC-style
   // permanent foothold trust), and a mid-strike probability collapse (the joint-torque switch's
   // CoP-under-ankle dropout, log 20260717_112516) cannot re-arm and double-fire within one touchdown.
   private static final double RESEED_TRIGGER_PROBABILITY = 0.5;
   private static final double RESEED_REARM_PROBABILITY = 0.1;
   private static final double RESEED_REARM_DWELL_SECONDS = 0.1; // genuine swing, not a mid-strike dropout
   private final SideDependentList<TouchdownReseedLatch> reseedLatches;
   private final YoBoolean yoReseedEnabled = new YoBoolean("invariantContactReseedEnabled", registry);
   private final SideDependentList<YoBoolean> yoReseedArmed;
   private final SideDependentList<YoInteger> yoReseedCount;
   private final SideDependentList<YoDouble> yoReseedResidual;

   // Gravity-leveling (tilt) diagnostics — the hardware-available "is pitch/roll wrong" signal (no ground truth
   // needed): the angle/roll/pitch between the measured gravity direction and the filter's estimate, computed
   // EVERY tick even when the update is gated off. yoGravityUpdateActive flags the ticks the tilt update fired.
   private final YoDouble yoGravityTiltErrorAngle = new YoDouble("invariantGravityTiltErrorAngle", registry);
   private final YoDouble yoGravityTiltErrorPitch = new YoDouble("invariantGravityTiltErrorPitch", registry);
   private final YoDouble yoGravityTiltErrorRoll = new YoDouble("invariantGravityTiltErrorRoll", registry);
   private final YoBoolean yoGravityUpdateActive = new YoBoolean("invariantGravityUpdateActive", registry);
   private final YoBoolean yoGravityPitchObservable = new YoBoolean("invariantGravityPitchObservable", registry);
   /** Watchdog: seconds since the tilt update last fired. A corrector that stops firing must be visible — the
    *  old gate latched off silently for the last 247 s of hardware log 20260712_163634 (FINDINGS.md §F.3). */
   private final YoDouble yoGravitySecondsSinceUpdate = new YoDouble("invariantGravitySecondsSinceUpdate", registry);

   // Per-foot contact-update conditioning diagnostics (mirrors the JointKF S-conditioning): cond(S) proxy (log10),
   // residual norm, and whether the update was applied (the conditioning gate can skip it). Plus a global
   // gate-skip counter across contact + gravity updates.
   private final SideDependentList<YoDouble> yoContactCondSProxyLog10;
   private final SideDependentList<YoDouble> yoContactResidualNorm;
   private final SideDependentList<YoBoolean> yoContactUpdateApplied;
   private final YoInteger yoInvariantUpdateGateSkipCount = new YoInteger("invariantUpdateGateSkipCount", registry);

   // The gyro bias actually subtracted from the raw gyro this tick, AFTER the MAX_GYRO_BIAS clamp, plus a count
   // of ticks the clamp bound. This is the "is the upstream bias sane" diagnostic.
   //
   // Published in BOTH frames, with the frame in the name. The pelvis IMU is yawed +90 deg about Z relative to
   // the pelvis, so the IMU-frame axes are NOT the pelvis axes of the same name (pelvis roll = -imu_Y, pelvis
   // pitch = +imu_X). Every one of these variables used to be IMU-frame with nothing in the name saying so,
   // sitting directly beside pelvis-frame rates -- which is exactly how a pitch bias gets read as a roll bias.
   // The bias is ESTIMATED in the IMU frame (that is where it physically lives), so that stays the primary; the
   // pelvis-frame copy is what you want when correlating against roll/pitch drift.
   private final YoDouble yoAppliedGyroBiasImuX = new YoDouble("invariantAppliedGyroBiasInIMUFrameX", registry);
   private final YoDouble yoAppliedGyroBiasImuY = new YoDouble("invariantAppliedGyroBiasInIMUFrameY", registry);
   private final YoDouble yoAppliedGyroBiasImuZ = new YoDouble("invariantAppliedGyroBiasInIMUFrameZ", registry);
   private final YoDouble yoAppliedGyroBiasPelvisX = new YoDouble("invariantAppliedGyroBiasInPelvisFrameX", registry);
   private final YoDouble yoAppliedGyroBiasPelvisY = new YoDouble("invariantAppliedGyroBiasInPelvisFrameY", registry);
   private final YoDouble yoAppliedGyroBiasPelvisZ = new YoDouble("invariantAppliedGyroBiasInPelvisFrameZ", registry);
   // Frame-invariant, so it needs no frame in the name and the old name is kept.
   private final YoDouble yoAppliedGyroBiasNorm = new YoDouble("invariantAppliedGyroBiasNorm", registry);
   private final YoInteger yoGyroBiasClampCount = new YoInteger("invariantGyroBiasClampCount", registry);

   // The RAW (un-bias-corrected) gyro expressed in the PELVIS frame -- the apples-to-apples partner for
   // invariantRootAngularVelocityBody{X,Y,Z}. Without this, the only raw gyro in the log is IMU-frame
   // (gyroscope_pelvis_imu*, pelvis_imuAngularVelocityInIMUFrame*), and comparing either of those against the
   // pelvis-frame estimate shows the mount rotation, not an estimator error: invariant_X tracks -raw_Y, so the
   // two "agree" in sign only when raw_X*raw_Y < 0, i.e. it flips with the motion quadrant and reads as an
   // intermittent, configuration-dependent sign error. It is not one. Diff THESE two and you see the bias, and
   // nothing else.
   private final YoDouble yoRawAngularVelocityBodyX = new YoDouble("invariantRawAngularVelocityBodyX", registry);
   private final YoDouble yoRawAngularVelocityBodyY = new YoDouble("invariantRawAngularVelocityBodyY", registry);
   private final YoDouble yoRawAngularVelocityBodyZ = new YoDouble("invariantRawAngularVelocityBodyZ", registry);

   // Estimate outputs for logging/comparison.
   private final YoFramePoint3D yoBasePosition = new YoFramePoint3D("invariantFilterPelvisBasePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoQuaternion yoBaseOrientation = new YoQuaternion("invariantFilterPelvisBaseOrientation", registry);
   private final YoFrameVector3D yoBaseVelocity = new YoFrameVector3D("invariantFilterPelvisBaseVelocity", ReferenceFrame.getWorldFrame(), registry);

   // Body-frame (pelvis-frame) views of the estimate. RPY is the estimated base orientation as
   // yaw-pitch-roll (world->body), addressing the earlier TODO for a controller-convention comparison; the
   // body velocity is the world base velocity rotated into the estimated pelvis frame (Rᵀ v). These are set
   // raw from the filter's own rotation, so the YoFrameVector3D is world-frame-labelled but holds body-frame
   // numbers (the name carries the frame), matching the existing "...Body" comparison variables.
   private final YoFrameYawPitchRoll yoBaseOrientationRPY = new YoFrameYawPitchRoll("invariantFilterPelvisOrientation",
                                                                                    ReferenceFrame.getWorldFrame(),
                                                                                    registry);
   private final YoFrameVector3D yoBaseVelocityBody = new YoFrameVector3D("invariantFilterPelvisBaseVelocityBody",
                                                                          ReferenceFrame.getWorldFrame(),
                                                                          registry);

   // 1-sigma covariance envelope around the base estimate: upper/lower = estimate +/- sqrt(diag(P)), read
   // from the filter's tangent-ordered covariance [δφ; δv; δp; …]. Position and velocity get a full
   // upper/lower envelope in the world frame; orientation exposes its per-axis tangent standard deviation
   // (rad) directly, since a +/- band on a rotation is not a meaningful world-frame vector.
   private final YoFramePoint3D yoBasePositionUpperBound = new YoFramePoint3D("invariantFilterPelvisBasePositionUpperBound",
                                                                              ReferenceFrame.getWorldFrame(),
                                                                              registry);
   private final YoFramePoint3D yoBasePositionLowerBound = new YoFramePoint3D("invariantFilterPelvisBasePositionLowerBound",
                                                                              ReferenceFrame.getWorldFrame(),
                                                                              registry);
   private final YoFrameVector3D yoBaseVelocityUpperBound = new YoFrameVector3D("invariantFilterPelvisBaseVelocityUpperBound",
                                                                                ReferenceFrame.getWorldFrame(),
                                                                                registry);
   private final YoFrameVector3D yoBaseVelocityLowerBound = new YoFrameVector3D("invariantFilterPelvisBaseVelocityLowerBound",
                                                                                ReferenceFrame.getWorldFrame(),
                                                                                registry);
   private final YoFrameVector3D yoBaseOrientationStandardDeviation = new YoFrameVector3D("invariantFilterPelvisOrientationStdDev",
                                                                                          ReferenceFrame.getWorldFrame(),
                                                                                          registry);

   // Per-tick temporary for the body-frame base velocity (Rᵀ v); pre-allocated to keep doControl allocation-free.
   private final Vector3D invariantLinearVelocityBody = new Vector3D();

   // Main (DRC) estimator's pelvis pose, kept only for the 3D marker (position is unobservable so it drifts).
   private final YoFramePoint3D yoMainBasePosition = new YoFramePoint3D("mainFilterPelvisBasePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoQuaternion yoMainBaseOrientation = new YoQuaternion("mainFilterPelvisBaseOrientation", registry);

   // Observable comparisons against the DRC estimator: orientation, and root-joint linear/angular velocity.
   private final YoDouble yoBaseOrientationErrorAngle = new YoDouble("invariantMinusMainOrientationErrorAngle", registry);

   private final YoFrameVector3D yoMainRootLinearVelocity = new YoFrameVector3D("mainRootLinearVelocityWorld", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D yoInvariantRootLinearVelocity = new YoFrameVector3D("invariantRootLinearVelocityWorld",
                                                                                     ReferenceFrame.getWorldFrame(),
                                                                                     registry);
   private final YoDouble yoLinearVelocityErrorMagnitude = new YoDouble("invariantMinusMainLinearVelocityErrorMagnitude", registry);

   // Body-frame numbers stored in world-frame-labelled vars (set raw); names carry the frame.
   private final YoFrameVector3D yoMainRootAngularVelocity = new YoFrameVector3D("mainRootAngularVelocityBody", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D yoInvariantRootAngularVelocity = new YoFrameVector3D("invariantRootAngularVelocityBody",
                                                                                      ReferenceFrame.getWorldFrame(),
                                                                                      registry);
   private final YoDouble yoAngularVelocityErrorMagnitude = new YoDouble("invariantMinusMainAngularVelocityErrorMagnitude", registry);

   //TODO: add YoVariable of body-frame RPY for comparison to controller convention

   /**
    * @param fullRobotModel             the (shared) estimator robot model used for forward kinematics.
    * @param sensorOutputMap            processed sensor outputs (IMU + joints).
    * @param primaryImuName             sensor name of the pelvis (base) IMU to use, e.g.
    *                                   {@code sensorInformation.getPrimaryBodyImu()}. Matched against
    *                                   {@link IMUSensorReadOnly#getSensorName()} in {@code sensorOutputMap}.
    * @param dt                         estimator timestep Δt (s).
    * @param gyroVariance               continuous gyro noise variance σ_ω² for the propagator.
    * @param accelVariance              continuous accel noise variance σ_a² for the propagator.
    * @param contactVariance            continuous contact-slip noise variance σ_c² for the propagator.
    * @param contactMeasurementVariance per-axis body-frame contact measurement variance (m²).
    * @param initialCovariance          scalar for the initial P = initialCovariance · I.
    * @param gravitationalAcceleration  the robot process's gravitational acceleration (m/s²), i.e.
    *                                   {@code AvatarEstimatorThreadFactory.getGravity()}. Sign not considered.
    */
   public InvariantEKFStateEstimator(FullHumanoidRobotModel fullRobotModel,
                                     SensorOutputMapReadOnly sensorOutputMap,
                                     String primaryImuName,
                                     double dt,
                                     double gyroVariance,
                                     double accelVariance,
                                     double contactVariance,
                                     double contactMeasurementVariance,
                                     double initialCovariance,
                                     double gravitationalAcceleration)
   {
      this(fullRobotModel,
           sensorOutputMap,
           primaryImuName,
           new ZeroIMUBiasProvider(), // no upstream bias estimation: subtracting an exact zero changes no arithmetic
           dt,
           gyroVariance,
           accelVariance,
           contactVariance,
           contactMeasurementVariance,
           initialCovariance,
           gravitationalAcceleration);
   }

   /**
    * @param imuBiasProvider upstream bias estimates for the primary IMU (e.g. the joint-level
    *                        pre-filter, or {@link ZeroIMUBiasProvider} for none). Biases are
    *                        subtracted from the raw measurements in the IMU measurement frame,
    *                        before the pelvis-frame transform.
    */
   public InvariantEKFStateEstimator(FullHumanoidRobotModel fullRobotModel,
                                     SensorOutputMapReadOnly sensorOutputMap,
                                     String primaryImuName,
                                     IMUBiasProvider imuBiasProvider,
                                     double dt,
                                     double gyroVariance,
                                     double accelVariance,
                                     double contactVariance,
                                     double contactMeasurementVariance,
                                     double initialCovariance,
                                     double gravitationalAcceleration)
   {
      this.dt = dt;
      this.initialCovariance = initialCovariance;
      this.sensorOutputMap = sensorOutputMap;
      this.imuBiasProvider = Objects.requireNonNull(imuBiasProvider, "imuBiasProvider must not be null (use ZeroIMUBiasProvider)");

      ekf = new InvariantEKF(NUMBER_OF_CONTACTS, gyroVariance, accelVariance, contactVariance, gravitationalAcceleration);

      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      pelvisFrame = referenceFrames.getPelvisFrame();
      for (RobotSide side : RobotSide.values)
         soleFrames.put(side, referenceFrames.getSoleFrame(side));

      Objects.requireNonNull(primaryImuName, "primaryImuName must not be null");
      imuSensor = sensorOutputMap.getIMUOutputs()
                                 .stream()
                                 .filter(imu -> imu.getSensorName().equals(primaryImuName))
                                 .findFirst()
                                 .orElseThrow(() -> new IllegalArgumentException("No IMU named '" + primaryImuName
                                                                                 + "' found in sensor output map. Available: "
                                                                                 + sensorOutputMap.getIMUOutputs()
                                                                                                   .stream()
                                                                                                   .map(IMUSensorReadOnly::getSensorName)
                                                                                                   .toList()));

      contactMeasurementNoiseProvider = new ConstantContactMeasurementNoiseProvider(contactMeasurementVariance);

      this.baseContactVariance = contactVariance;
      yoContactProbability = new SideDependentList<>(new YoDouble("invariantContactProbabilityLeft", registry),
                                                     new YoDouble("invariantContactProbabilityRight", registry));

      yoContactNIS = new SideDependentList<>(new YoDouble("invariantContactNISLeft", registry),
                                             new YoDouble("invariantContactNISRight", registry));
      yoContactCondSProxyLog10 = new SideDependentList<>(new YoDouble("invariantContactCondSProxyLog10Left", registry),
                                                         new YoDouble("invariantContactCondSProxyLog10Right", registry));
      yoContactResidualNorm = new SideDependentList<>(new YoDouble("invariantContactResidualNormLeft", registry),
                                                      new YoDouble("invariantContactResidualNormRight", registry));
      yoContactUpdateApplied = new SideDependentList<>(new YoBoolean("invariantContactUpdateAppliedLeft", registry),
                                                       new YoBoolean("invariantContactUpdateAppliedRight", registry));
      yoContactSHPHtTrace = new SideDependentList<>(new YoDouble("invariantContactSHPHtTraceLeft", registry),
                                                    new YoDouble("invariantContactSHPHtTraceRight", registry));
      yoContactSRTrace = new SideDependentList<>(new YoDouble("invariantContactSRTraceLeft", registry),
                                                 new YoDouble("invariantContactSRTraceRight", registry));
      yoContactRInflation = new SideDependentList<>(new YoDouble("invariantContactRInflationLeft", registry),
                                                    new YoDouble("invariantContactRInflationRight", registry));
      yoContactPContactTrace = new SideDependentList<>(new YoDouble("invariantContactPContactTraceLeft", registry),
                                                       new YoDouble("invariantContactPContactTraceRight", registry));
      yoContactCorrectionRotNorm = new SideDependentList<>(new YoDouble("invariantContactCorrectionRotNormLeft", registry),
                                                           new YoDouble("invariantContactCorrectionRotNormRight", registry));
      yoContactCorrectionVelNorm = new SideDependentList<>(new YoDouble("invariantContactCorrectionVelNormLeft", registry),
                                                           new YoDouble("invariantContactCorrectionVelNormRight", registry));
      yoContactCorrectionPosNorm = new SideDependentList<>(new YoDouble("invariantContactCorrectionPosNormLeft", registry),
                                                           new YoDouble("invariantContactCorrectionPosNormRight", registry));
      yoReseedArmed = new SideDependentList<>(new YoBoolean("invariantContactReseedArmedLeft", registry),
                                              new YoBoolean("invariantContactReseedArmedRight", registry));
      yoReseedCount = new SideDependentList<>(new YoInteger("invariantContactReseedCountLeft", registry),
                                              new YoInteger("invariantContactReseedCountRight", registry));
      yoReseedResidual = new SideDependentList<>(new YoDouble("invariantContactReseedResidualLeft", registry),
                                                 new YoDouble("invariantContactReseedResidualRight", registry));
      yoReseedEnabled.set(true);
      int rearmDwellTicks = Math.max(1, (int) Math.round(RESEED_REARM_DWELL_SECONDS / dt));
      reseedLatches = new SideDependentList<>(new TouchdownReseedLatch(RESEED_TRIGGER_PROBABILITY, RESEED_REARM_PROBABILITY, rearmDwellTicks, false),
                                              new TouchdownReseedLatch(RESEED_TRIGGER_PROBABILITY, RESEED_REARM_PROBABILITY, rearmDwellTicks, false));
      for (RobotSide side : RobotSide.values)
         yoReseedArmed.get(side).set(false); // arm only after a clean sustained swing (prob < re-arm threshold for the dwell)
      // Two-sided χ² acceptance band for the contact-update NIS (constant: fixed DOF and confidence).
      ChiSquaredDistribution contactNISDistribution = new ChiSquaredDistribution(CONTACT_MEASUREMENT_DOF);
      double lowerTail = 0.5 * (1.0 - CONSISTENCY_CONFIDENCE);
      yoContactNISLowerBound.set(contactNISDistribution.inverseCumulativeProbability(lowerTail));
      yoContactNISUpperBound.set(contactNISDistribution.inverseCumulativeProbability(1.0 - lowerTail));
      // Default fallback: forward-kinematics-only contact detection on the estimator's own sole frames
      // (already refreshed each tick in doControl, so no frame-updater hook is needed here).
      contactProbabilityProvider = new KinematicContactDetector(soleFrames, null, dt);
   }

   @Override
   public void initializeEstimator(RigidBodyTransformReadOnly rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
      referenceFrames.updateFrames();

      tempRotation.set(rootJointTransform.getRotation());
      Point3D basePosition = new Point3D(rootJointTransform.getTranslation());

      Tuple3DReadOnly[] contactPositions = new Tuple3DReadOnly[NUMBER_OF_CONTACTS];
      for (RobotSide side : RobotSide.values)
      {
         contactInWorld.setToZero(soleFrames.get(side));
         contactInWorld.changeFrame(ReferenceFrame.getWorldFrame());
         contactPositions[CONTACT_INDICES.get(side)] = new Point3D(contactInWorld);
      }

      ekf.initialize(tempRotation, new Vector3D(), basePosition, contactPositions, scaledIdentity(initialCovariance));
      updateYoVariables();
   }

   /**
    * Re-seed the flter when resuming from a held state: base pose from the current (gyro-tracked) pelvis frame,
    * contact anchors from current sole FK, zero velocity, covariance reset to P = initialCovariance * I.
    * Runs only on the hold->active transition, so the allication here is not when the estimator is actively running yet.
    */
   /**
    * Replaces the contact FK measurement noise source (default: the constant isotropic diagonal).
    * This is the step-8 injection point for a covariance-routing provider (J Sigma_q J^T from a
    * joint-level pre-filter with {@code hasCovariance()}).
    */
   public void setContactMeasurementNoiseProvider(ContactMeasurementNoiseProvider contactMeasurementNoiseProvider)
   {
      this.contactMeasurementNoiseProvider = Objects.requireNonNull(contactMeasurementNoiseProvider);
   }

   public void reAnchor()
   {
      referenceFrames.updateFrames();

      mainEstimatePelvisPose.setToZero(pelvisFrame);
      mainEstimatePelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      tempRotation.set(mainEstimatePelvisPose.getOrientation());
      Point3D basePosition = new Point3D(mainEstimatePelvisPose.getPosition());

      Tuple3DReadOnly[] contactPositions = new Tuple3DReadOnly[NUMBER_OF_CONTACTS];
      for (RobotSide side: RobotSide.values)
      {
         contactInWorld.setToZero(soleFrames.get(side));
         contactInWorld.changeFrame(ReferenceFrame.getWorldFrame());
         contactPositions[CONTACT_INDICES.get(side)] = new Point3D(contactInWorld);
      }

      ekf.initialize(tempRotation, new Vector3D(), basePosition, contactPositions, scaledIdentity(initialCovariance));
      updateYoVariables();
   }

   @Override
   public void doControl()
   {
      referenceFrames.updateFrames();

      // Refresh per-foot contact probabilities, then apply knob 2 (contact-position process noise) before
      // the prediction's covariance step consumes Q_c: a swing foot's anchor is given large slip noise so
      // it re-anchors softly on touchdown instead of kicking the base.
      contactProbabilityProvider.update();
      for (RobotSide side : RobotSide.values)
      {
         double contactProbability = clamp(contactProbabilityProvider.getContactProbability(side));
         yoContactProbability.get(side).set(contactProbability);
         ekf.setContactSlipVariance(CONTACT_INDICES.get(side), contactSlipVariance(contactProbability));
      }

      // IMU omega and a: bias-corrected in the measurement frame (where the bias is estimated),
      // THEN expressed in the pelvis frame. Frame-checked sub() throws on a wrong-frame provider.
      angularVelocity.setIncludingFrame(imuSensor.getMeasurementFrame(), imuSensor.getAngularVelocityMeasurement());
      // Clamp the upstream gyro bias to a physically plausible range before subtracting: a runaway bias would
      // otherwise be integrated straight into base orientation (the pitch-drift mechanism). Publish the applied
      // (post-clamp) bias for visibility.
      appliedGyroBias.setIncludingFrame(imuSensor.getMeasurementFrame(), imuBiasProvider.getAngularVelocityBiasInIMUFrame(imuSensor));
      boolean clamped = clampGyroBias(appliedGyroBias);
      if (clamped)
         yoGyroBiasClampCount.set(yoGyroBiasClampCount.getValue() + 1);
      yoAppliedGyroBiasImuX.set(appliedGyroBias.getX());
      yoAppliedGyroBiasImuY.set(appliedGyroBias.getY());
      yoAppliedGyroBiasImuZ.set(appliedGyroBias.getZ());
      yoAppliedGyroBiasNorm.set(appliedGyroBias.norm()); // frame-invariant
      // Same bias in the pelvis frame, for correlating against roll/pitch drift. Uses its own scratch: rotating
      // appliedGyroBias itself would break the frame-checked sub() below, which needs it in the IMU frame.
      appliedGyroBiasInPelvis.setIncludingFrame(appliedGyroBias);
      appliedGyroBiasInPelvis.changeFrame(pelvisFrame);
      yoAppliedGyroBiasPelvisX.set(appliedGyroBiasInPelvis.getX());
      yoAppliedGyroBiasPelvisY.set(appliedGyroBiasInPelvis.getY());
      yoAppliedGyroBiasPelvisZ.set(appliedGyroBiasInPelvis.getZ());
      // Keep the RAW gyro too: the quasi-static gate and its gravity reference must be built from sensors only,
      // never from a filter state or an upstream bias that could itself be diverging (FINDINGS.md §F.3).
      rawAngularVelocity.setIncludingFrame(imuSensor.getMeasurementFrame(), imuSensor.getAngularVelocityMeasurement());
      rawAngularVelocity.changeFrame(pelvisFrame);
      // Publish it: this is the pelvis-frame raw gyro, the ONLY correct thing to diff against
      // invariantRootAngularVelocityBody*. Their difference is exactly the applied bias, by construction.
      yoRawAngularVelocityBodyX.set(rawAngularVelocity.getX());
      yoRawAngularVelocityBodyY.set(rawAngularVelocity.getY());
      yoRawAngularVelocityBodyZ.set(rawAngularVelocity.getZ());
      angularVelocity.sub(appliedGyroBias);
      angularVelocity.changeFrame(pelvisFrame);
      linearAcceleration.setIncludingFrame(imuSensor.getMeasurementFrame(), imuSensor.getLinearAccelerationMeasurement());
      linearAcceleration.sub(imuBiasProvider.getLinearAccelerationBiasInIMUFrame(imuSensor));
      linearAcceleration.changeFrame(pelvisFrame);

      // Advance the gate's sensor-only gravity reference every tick, on both the held and the normal path.
      ekf.updateGravityReference(linearAcceleration, rawAngularVelocity, dt);

      boolean noContact = getContactProbability(RobotSide.LEFT) < CONTACT_HOLD_THRESHOLD && getContactProbability(RobotSide.RIGHT) < CONTACT_HOLD_THRESHOLD;

      if (operatingMode == StateEstimatorMode.FROZEN || noContact)
      {
         // Base translation is unobservable (frozen/hanging). Integrate
         // gyro as usual, but hold translation: zero the base velocity so the accel/gravity residual can't ramp
         // it (the -2 m/s drift in Z while hanging). Contact updates skipped by returning
         ekf.predict(angularVelocity, linearAcceleration, dt);
         // Gravity leveling is orientation-only (H has no translation columns), so it is safe while base
         // translation is held — and it is the ONLY roll/pitch observation available while hanging/no-contact.
         updateGravityLeveling();
         ekf.getState().setBaseVelocity(zeroVelocity);
         heldLastTick = true;
         for (RobotSide side : RobotSide.values)
         {
            yoContactNIS.get(side).setToNaN();
            yoContactUpdateApplied.get(side).set(false);
         }
         updateYoVariables();
         return;
      }

      if (heldLastTick)
      {
         // First observable tick after a hold, feet have moved, so the contact anchors are stale
         // Need to re-ssed anchors form current FK before using contact update appropriately
         reAnchor();
         heldLastTick = false;
      }

      ekf.predict(angularVelocity, linearAcceleration, dt);

      // Gravity-leveling: the roll/pitch anchor the contact update cannot provide (its Jacobian has a zero
      // rotation block). Diagnostic refreshed every tick; update applied only when quasi-static.
      updateGravityLeveling();

      // Knob 1 (measurement covariance): inflate a swing foot's contact FK noise so it stops dragging the
      // base velocity, while a stance foot keeps constraining it.
      yoContactPBaseTrace.set(covarianceBlockTrace(ekf.getState().basePositionTangentIndex()));
      for (RobotSide side : RobotSide.values)
      {
         double contactProbability = yoContactProbability.get(side).getDoubleValue();
         contactInBody.setToZero(soleFrames.get(side));
         contactInBody.changeFrame(pelvisFrame);
         contactMeasurementNoiseProvider.packContactCovariance(side, inflatedContactCovariance);

         // H4 Phase 2: one-shot touchdown re-seed, BEFORE this side's update consumes the residual.
         // Uses the UN-inflated FK covariance (the re-seeded anchor is as good as the FK that placed it).
         if (yoReseedEnabled.getBooleanValue())
         {
            if (reseedLatches.get(side).advance(contactProbability))
            {
               yoReseedResidual.get(side).set(ekf.reseedContact(CONTACT_INDICES.get(side), contactInBody, inflatedContactCovariance));
               yoReseedCount.get(side).increment();
            }
            yoReseedArmed.get(side).set(reseedLatches.get(side).isArmed());
         }

         double inflation = measurementInflation(contactProbability);
         inflatedContactCovariance.scale(inflation);
         yoContactRInflation.get(side).set(inflation);
         yoContactPContactTrace.get(side).set(covarianceBlockTrace(ekf.getState().contactTangentIndex(CONTACT_INDICES.get(side))));
         ekf.update(CONTACT_INDICES.get(side), contactInBody, inflatedContactCovariance);
         yoContactNIS.get(side).set(ekf.getLastNormalizedInnovationSquared());
         yoContactCondSProxyLog10.get(side).set(Math.log10(ekf.getLastConditionProxy()));
         yoContactResidualNorm.get(side).set(ekf.getLastResidualNorm());
         yoContactUpdateApplied.get(side).set(ekf.wasLastUpdateApplied());
         yoContactSHPHtTrace.get(side).set(ekf.getLastHPHtTrace());
         yoContactSRTrace.get(side).set(ekf.getLastMeasurementNoiseTrace());
         yoContactCorrectionRotNorm.get(side).set(ekf.getLastCorrectionRotationNorm());
         yoContactCorrectionVelNorm.get(side).set(ekf.getLastCorrectionVelocityNorm());
         yoContactCorrectionPosNorm.get(side).set(ekf.getLastCorrectionPositionNorm());
      }
      yoInvariantUpdateGateSkipCount.set(ekf.getUpdateGateSkipCount());

      updateYoVariables();
   }

   /** Trace of the 3×3 diagonal block of the EKF covariance starting at {@code tangentIndex}. */
   private double covarianceBlockTrace(int tangentIndex)
   {
      org.ejml.data.DMatrixRMaj covariance = ekf.getState().getCovariance();
      return covariance.get(tangentIndex, tangentIndex)
           + covariance.get(tangentIndex + 1, tangentIndex + 1)
           + covariance.get(tangentIndex + 2, tangentIndex + 2);
   }

   /**
    * Refreshes the gravity-leveling tilt diagnostic (every tick) and applies the accelerometer tilt update when
    * enabled and quasi-static. The tilt diagnostic — the angle/roll/pitch between the measured gravity direction
    * and the filter's estimate — is a hardware-available "is pitch/roll wrong" signal that needs no ground truth
    * (unlike the sim-only ground-truth comparator, and unlike invariantMinusMain* which is NaN when the invariant
    * filter is the main estimator). Call right after {@link InvariantEKF#predict}.
    */
   private void updateGravityLeveling()
   {
      // Pitch is leveled only when observable: always in double support; in single support only if the
      // double-support pitch gate is off. Roll is always leveled. Set before assemble() (it builds R).
      boolean doubleSupport = getContactProbability(RobotSide.LEFT) >= DOUBLE_SUPPORT_CONTACT_PROBABILITY
                           && getContactProbability(RobotSide.RIGHT) >= DOUBLE_SUPPORT_CONTACT_PROBABILITY;
      boolean pitchObservable = !gatePitchOnDoubleSupport || doubleSupport;
      ekf.setGravityPitchObservable(pitchObservable);
      yoGravityPitchObservable.set(pitchObservable);

      ekf.assembleGravityLeveling(linearAcceleration);
      yoGravityTiltErrorAngle.set(ekf.getGravityTiltErrorAngle());
      yoGravityTiltErrorPitch.set(ekf.getGravityTiltErrorPitch());
      yoGravityTiltErrorRoll.set(ekf.getGravityTiltErrorRoll());

      boolean apply = gravityLevelingEnabled
            && ekf.isGravityQuasiStatic(linearAcceleration,
                                        rawAngularVelocity,
                                        QUASI_STATIC_ACCEL_TOLERANCE,
                                        QUASI_STATIC_GYRO_THRESHOLD,
                                        QUASI_STATIC_HORIZONTAL_ACCEL_THRESHOLD);
      yoGravityUpdateActive.set(apply);
      if (apply)
      {
         ekf.applyGravityLeveling();
         yoGravitySecondsSinceUpdate.set(0.0);
      }
      else
      {
         yoGravitySecondsSinceUpdate.add(dt);
      }
   }

   /** Enables/disables the accelerometer gravity-leveling (tilt) update. The tilt diagnostic is published regardless. */
   public void setGravityLevelingEnabled(boolean gravityLevelingEnabled)
   {
      this.gravityLevelingEnabled = gravityLevelingEnabled;
   }

   /**
    * If true, the PITCH leveling is applied only in double support (roll is always leveled). Default false.
    * Experimental knob to try if the anisotropic σ_pitch + horizontal-accel gate leave a residual forward lean;
    * off by default so single-support pitch stays observable for diagnosis rather than masked.
    */
   public void setGatePitchOnDoubleSupport(boolean gatePitchOnDoubleSupport)
   {
      this.gatePitchOnDoubleSupport = gatePitchOnDoubleSupport;
   }

   /** Clamps each gyro-bias component to ±{@link #MAX_GYRO_BIAS}; returns true if any axis was clamped. */
   private static boolean clampGyroBias(FrameVector3D bias)
   {
      double x = clampComponent(bias.getX());
      double y = clampComponent(bias.getY());
      double z = clampComponent(bias.getZ());
      boolean clamped = x != bias.getX() || y != bias.getY() || z != bias.getZ();
      bias.set(x, y, z);
      return clamped;
   }

   private static double clampComponent(double v)
   {
      return Math.max(-MAX_GYRO_BIAS, Math.min(MAX_GYRO_BIAS, v));
   }

   private void updateYoVariables()
   {
      ekf.getBasePosition(tempVector);
      yoBasePosition.set(tempVector);

      ekf.getRotation(tempRotation);
      yoBaseOrientation.set(tempRotation);

      ekf.getBaseVelocity(invariantLinearVelocityWorld);
      yoBaseVelocity.set(invariantLinearVelocityWorld);

      // Body-frame views + covariance envelope. Valid regardless of runningAsMain (they are the filter's own
      // estimate, not a cross-estimator comparison), so they are set here before the runningAsMain branch.
      updateBodyFrameYoVariables();
      updateCovarianceBoundYoVariables();

      // Main estimator's pelvis pose = the shared model's pelvis frame (the main estimator ticks first).
      // Position is unobservable here, so it is kept only for the 3D marker; orientation is observable.
      mainEstimatePelvisPose.setToZero(pelvisFrame);
      mainEstimatePelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      yoMainBasePosition.set(mainEstimatePelvisPose.getPosition());
      yoMainBaseOrientation.set(mainEstimatePelvisPose.getOrientation());

      if (runningAsMain)
      {
         // The shared pelvis frame is this filter's own previous-tick output (the root joint is written
         // after this update), so a "main" comparison here is a one-tick-stale self-comparison. NaN the
         // comparison variables; ground-truth comparisons live in the sim-side comparator instead.
         yoBaseOrientationErrorAngle.set(Double.NaN);
         yoMainRootLinearVelocity.setToNaN();
         yoLinearVelocityErrorMagnitude.set(Double.NaN);
         yoMainRootAngularVelocity.setToNaN();
         yoAngularVelocityErrorMagnitude.set(Double.NaN);

         yoInvariantRootLinearVelocity.set(invariantLinearVelocityWorld);
         invariantAngularVelocityBody.set(angularVelocity);
         yoInvariantRootAngularVelocity.set(invariantAngularVelocityBody);
         return;
      }

      yoBaseOrientationErrorAngle.set(yoBaseOrientation.distance(mainEstimatePelvisPose.getOrientation()));

      // DRC estimator's estimated root-joint twist = the shared model's pelvis-frame twist.
      TwistReadOnly pelvisTwist = pelvisFrame.getTwistOfFrame();

      // Linear velocity (observable), compared in the world frame.
      mainLinearVelocity.setIncludingFrame(pelvisTwist.getLinearPart());
      mainLinearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      yoMainRootLinearVelocity.set(mainLinearVelocity);
      yoInvariantRootLinearVelocity.set(invariantLinearVelocityWorld);
      velocityErrorTemp.sub(invariantLinearVelocityWorld, mainLinearVelocity);
      yoLinearVelocityErrorMagnitude.set(velocityErrorTemp.norm());

      // Angular velocity, compared in the pelvis (body) frame: the invariant filter's angular rate is the
      // gyro it integrates; the DRC value is the twist's angular part (both already in the pelvis frame).
      mainAngularVelocityBody.set(pelvisTwist.getAngularPart());
      invariantAngularVelocityBody.set(angularVelocity);
      yoMainRootAngularVelocity.set(mainAngularVelocityBody);
      yoInvariantRootAngularVelocity.set(invariantAngularVelocityBody);
      velocityErrorTemp.sub(invariantAngularVelocityBody, mainAngularVelocityBody);
      yoAngularVelocityErrorMagnitude.set(velocityErrorTemp.norm());
   }

   /**
    * Publishes the body-frame (pelvis-frame) views of the estimate: the base orientation as yaw-pitch-roll,
    * and the base velocity rotated from world into the estimated pelvis frame (v_body = Rᵀ v_world). Reads
    * {@code tempRotation} and {@code invariantLinearVelocityWorld}, both set at the top of
    * {@link #updateYoVariables()}. Allocation-free.
    */
   private void updateBodyFrameYoVariables()
   {
      yoBaseOrientationRPY.set(tempRotation);
      tempRotation.inverseTransform(invariantLinearVelocityWorld, invariantLinearVelocityBody);
      yoBaseVelocityBody.set(invariantLinearVelocityBody);
   }

   /**
    * Publishes the 1-sigma covariance envelope around the base estimate: position and velocity as
    * upper/lower = estimate +/- sqrt(diag(P)) in the world frame, and orientation as its per-axis tangent
    * standard deviation. Reads the tangent-ordered covariance and the base position/velocity set at the top
    * of {@link #updateYoVariables()}. Allocation-free.
    */
   private void updateCovarianceBoundYoVariables()
   {
      InvariantState state = ekf.getState();
      DMatrixRMaj covariance = state.getCovariance();
      int orientationIndex = state.rotationTangentIndex();
      int velocityIndex = state.baseVelocityTangentIndex();
      int positionIndex = state.basePositionTangentIndex();

      double sigmaPositionX = standardDeviation(covariance, positionIndex);
      double sigmaPositionY = standardDeviation(covariance, positionIndex + 1);
      double sigmaPositionZ = standardDeviation(covariance, positionIndex + 2);
      yoBasePositionUpperBound.set(tempVector.getX() + sigmaPositionX,
                                   tempVector.getY() + sigmaPositionY,
                                   tempVector.getZ() + sigmaPositionZ);
      yoBasePositionLowerBound.set(tempVector.getX() - sigmaPositionX,
                                   tempVector.getY() - sigmaPositionY,
                                   tempVector.getZ() - sigmaPositionZ);

      double sigmaVelocityX = standardDeviation(covariance, velocityIndex);
      double sigmaVelocityY = standardDeviation(covariance, velocityIndex + 1);
      double sigmaVelocityZ = standardDeviation(covariance, velocityIndex + 2);
      yoBaseVelocityUpperBound.set(invariantLinearVelocityWorld.getX() + sigmaVelocityX,
                                   invariantLinearVelocityWorld.getY() + sigmaVelocityY,
                                   invariantLinearVelocityWorld.getZ() + sigmaVelocityZ);
      yoBaseVelocityLowerBound.set(invariantLinearVelocityWorld.getX() - sigmaVelocityX,
                                   invariantLinearVelocityWorld.getY() - sigmaVelocityY,
                                   invariantLinearVelocityWorld.getZ() - sigmaVelocityZ);

      yoBaseOrientationStandardDeviation.set(standardDeviation(covariance, orientationIndex),
                                             standardDeviation(covariance, orientationIndex + 1),
                                             standardDeviation(covariance, orientationIndex + 2));
   }

   /** Diagonal standard deviation sqrt(P[index, index]), clamped at 0 to stay finite through numerical negatives. */
   private static double standardDeviation(DMatrixRMaj covariance, int index)
   {
      return Math.sqrt(Math.max(0.0, covariance.get(index, index)));
   }

   /**
    * Installs the contact-probability source (e.g. the controller-contact oracle, or a learned ContactNet
    * head). Replaces the default forward-kinematics detector.
    *
    * @param provider the new provider. Not null.
    */
   public void setContactProbabilityProvider(ContactProbabilityProvider provider)
   {
      this.contactProbabilityProvider = Objects.requireNonNull(provider, "contactProbabilityProvider");
   }

   /** @return the installed contact-probability source. */
   public ContactProbabilityProvider getContactProbabilityProvider()
   {
      return contactProbabilityProvider;
   }

   /**
    * Marks this filter as the main estimator (it drives the shared robot model's root joint). Disables the
    * {@code invariantMinusMain*} comparisons, which would otherwise be stale self-comparisons — see
    * {@link #runningAsMain}. Called by {@link InvariantMainStateEstimator}.
    */
   public void setRunningAsMain(boolean runningAsMain)
   {
      this.runningAsMain = runningAsMain;
   }

   /** @return whether this filter drives the shared robot model's root joint as the main estimator. */
   public boolean isRunningAsMain()
   {
      return runningAsMain;
   }

   /** Sets the measurement-covariance inflation a fully-swing foot (p = 0) receives; 1 disables knob 1. */
   public void setSwingMeasurementInflation(double swingMeasurementInflation)
   {
      this.swingMeasurementInflation = swingMeasurementInflation;
   }

   /** Sets the contact-slip-variance inflation a fully-swing foot (p = 0) receives; 1 disables knob 2. */
   public void setSwingSlipInflation(double swingSlipInflation)
   {
      this.swingSlipInflation = swingSlipInflation;
   }

   /**
    * @param side the foot to query.
    * @return the most recent contact probability p ∈ [0,1] for {@code side} (set each {@code doControl}).
    *         Exposed for the yaw-seeding corrector when this filter drives the base as the main estimator.
    */
   public double getContactProbability(RobotSide side)
   {
      return yoContactProbability.get(side).getDoubleValue();
   }

   /** Knob 1: contact FK measurement covariance scale, inflation^(1−p) — 1 at p = 1, swing inflation at p = 0. */
   private double measurementInflation(double contactProbability)
   {
      return Math.pow(swingMeasurementInflation, 1.0 - contactProbability);
   }

   /** Knob 2: contact-position slip variance σ_{c,i}², base × inflation^(1−p). */
   private double contactSlipVariance(double contactProbability)
   {
      return baseContactVariance * Math.pow(swingSlipInflation, 1.0 - contactProbability);
   }

   private static double clamp(double value)
   {
      return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
   }

   private static org.ejml.data.DMatrixRMaj scaledIdentity(double scale)
   {
      int m = 9 + 3 * NUMBER_OF_CONTACTS;
      org.ejml.data.DMatrixRMaj matrix = new org.ejml.data.DMatrixRMaj(m, m);
      for (int i = 0; i < m; i++)
         matrix.set(i, i, scale);
      return matrix;
   }

   @Override
   public void requestStateEstimatorMode(StateEstimatorMode operatingMode)
   {
      this.operatingMode = operatingMode; // used in doControl(); FROZEN->hold
   }

   @Override
   public void initialize()
   {
      referenceFrames.updateFrames();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public YoGraphicGroupDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(name);
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("invariantPelvisEstimate", yoBasePosition, 0.05, ColorDefinitions.Red()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("mainPelvisEstimate", yoMainBasePosition, 0.05, ColorDefinitions.Green()));
      return group;
   }

   /** @return the underlying filter, e.g. for tests or external inspection. */
   public InvariantEKF getInvariantEKF()
   {
      return ekf;
   }

   /**
    * @return the reference frames this estimator maintains on the shared robot model, refreshed at the
    *         top of every {@link #doControl()} <em>before</em> the contact-probability provider runs.
    *         Exposed so contactable feet / foot switches handed to
    *         {@link #setContactProbabilityProvider} can be built on frames that are guaranteed current
    *         when the provider is polled.
    */
   public HumanoidReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   /**
    * @return the shared estimator robot model's pelvis frame, i.e. the <em>main</em> (DRC) estimator's
    *         pelvis pose/twist. This is the same frame used above to compute
    *         {@code invariantMinusMainOrientationErrorAngle} and the main-side velocity comparisons.
    *         Exposed so external comparators (e.g. against simulation ground truth) can read it directly
    *         instead of re-deriving it from the robot model.
    */
   public MovingReferenceFrame getMainPelvisFrame()
   {
      return pelvisFrame;
   }

   /**
    * Packs the body-frame angular velocity the filter integrates (the bias-free IMU gyro in the pelvis
    * frame). Exposed so external comparators can compare it against a reference angular velocity.
    *
    * @param angularVelocityToPack the body-frame angular velocity. Modified.
    */
   public void getMeasuredAngularVelocityInBody(us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }
}

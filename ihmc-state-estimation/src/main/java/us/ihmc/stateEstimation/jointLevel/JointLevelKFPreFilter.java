package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUBasedJointStateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.ToDoubleFunction;

/**
 * Joint-level Kalman filter pre-filter (P-A architecture): one filter over the IMU tree whose pair
 * measurements z_w,ab = J(q^) S_ab qd + b_w,ab + v couple the joints on each IMU-pair path, with
 * per-IMU biases in state and stance-foot gauge anchors for the absolute base bias. When a robot model is
 * provided, the joint process noise is the mass-matrix-induced Qa = sigma_tau^2 M(q)^-2 of the write-up
 * (eqs. (10)-(12)), recomputed each predict; otherwise a scalar-CWNA diagonal fallback.
 *
 * <p><b>Do not share an instance between pipelines</b> (the filter holds its covariance).</p>
 *
 * <p>This class is the ORCHESTRATOR: it owns the tick sequence, the on-ground initialization gate, and the
 * {@code OneDoFJointStateSource} / {@code IMUBiasProvider} surface the estimator consumes. The filter itself is
 * five collaborators, constructed in this order and depending on each other one way only
 * (BiasUpdate → Update → State), so there are no initialization cycles: {@link JointKFParameters} (the tuning,
 * as YoVariables), {@link JointKFState} (layout, x and P, the structural holders), {@link JointKFPrediction}
 * (F, Q, the Schur mass-matrix noise), {@link JointKFUpdate} (encoder + direct-velocity channels and the shared
 * Joseph core), and {@link JointKFBiasUpdate} (the stacked gyro channel).</p>
 *
 * <p>Rev. 2 measurement model (SPEC §5-§6): the gyro measurements are ONE stacked Joseph update per tick over
 * [pair diffs ; active stance anchors], not Rev. 1's per-pair sequential updates, so R_g can carry the exact
 * shared-IMU cross-covariances. {@link #computeImuBiases} is reduced to caching the trusted-feet set.</p>
 *
 * <p>Rev. 2 process noise (SPEC §3.2): Qa is the floating-base Schur complement sigma_tau^2 Lambda(q)^-2 rather
 * than the locked-base sigma_tau^2 M_jj^-2. Because the free base recoils, Lambda ⪯ M_jj and so Lambda^-2 ⪰
 * M_jj^-2 — the effective process noise GROWS at fixed sigma_tau, worst for proximal joints, which is why
 * sigma_tau carries a retune obligation.</p>
 *
 * @author Lucas Libshutz
 */
public class JointLevelKFPreFilter implements ProprioceptivePreFilter
{
   private final JointKFParameters parameters;
   private final JointKFState state;
   private final JointKFPrediction prediction;
   private final JointKFUpdate update;
   private final JointKFBiasUpdate biasUpdate;

   // Optional on-ground gate (see JointKFParameters.ON_GROUND_INIT_DEBOUNCE). Null => no gating: the filter
   // initializes as soon as boot data is finite (the behavior the package tests rely on). Wired by the consumer
   // that owns the contact-probability signal (InvariantMainStateEstimator) via setInitializationGate.
   private BooleanSupplier onGroundGate = null;
   private int consecutiveOnGroundTicks = 0;
   private final int requiredOnGroundTicks;

   // ONE flat registry shared by every component, so YoVariable simple names stay unique and their full
   // namespaces are unchanged by the split. (This was once handed a parentRegistry that it dropped on the
   // floor, so the filter published nothing on hardware.)
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoBoolean yoInitialized = new YoBoolean("jointKFInitialized", registry);
   private final YoBoolean yoWaitingForGroundContact = new YoBoolean("jointKFWaitingForGroundContact", registry);

   private final FrameVector3D biasOut = new FrameVector3D();

   // Package-private (not private) so the allocation/behavior tests in this package can build it directly
   // from a synthetic IMU-pair setup without standing up a full StateEstimatorParameters.
   /** Scalar-CWNA process-noise overload: no robot model, so Qa = SIGMA_ACCEL^2 I (the pre-mass-matrix behavior). */
   //TODO: all objects passed in should be `final`, so they should either be declared in the constructor or the class definition.
   JointLevelKFPreFilter(SensorOutputMapReadOnly sensorMap,
                         List<IMUBasedJointStateEstimatorParameters> pairParameters,
                         Collection<RigidBodyBasics> feet,
                         double estimatorDT,
                         YoRegistry parentRegistry)
   {
      this(sensorMap, pairParameters, feet, null, null, null, null, false, Double.NaN, estimatorDT, parentRegistry);
   }

   /** Overload without per-joint encoder noise lookups: every joint uses the scalar fallbacks. */
   JointLevelKFPreFilter(SensorOutputMapReadOnly sensorMap,
                         List<IMUBasedJointStateEstimatorParameters> pairParameters,
                         Collection<RigidBodyBasics> feet,
                         RigidBodyBasics rootBody,
                         double estimatorDT,
                         YoRegistry parentRegistry)
   {
      this(sensorMap, pairParameters, feet, rootBody, null, null, null, false, Double.NaN, estimatorDT, parentRegistry);
   }

   /** Overload without the direct-velocity channel (per-joint encoder noise lookups only). */
   JointLevelKFPreFilter(SensorOutputMapReadOnly sensorMap,
                         List<IMUBasedJointStateEstimatorParameters> pairParameters,
                         Collection<RigidBodyBasics> feet,
                         RigidBodyBasics rootBody,
                         ToDoubleFunction<String> encoderPositionNoiseStd,
                         ToDoubleFunction<String> encoderVelocityNoiseStd,
                         double estimatorDT,
                         YoRegistry parentRegistry)
   {
      this(sensorMap, pairParameters, feet, rootBody, encoderPositionNoiseStd, encoderVelocityNoiseStd, null, false, Double.NaN, estimatorDT,
           parentRegistry);
   }

   /**
    * @param rootBody root of the estimator's robot model (the elevator). When non-null, the joint process
    *                 noise is the mass-matrix-induced Qa = sigma_tau^2 M(q)^-2 (write-up eqs. (10)-(12)),
    *                 recomputed every predict because M depends on the configuration. When null, falls back
    *                 to the constant scalar-CWNA diagonal Qa = SIGMA_ACCEL^2 I.
    * @param encoderPositionNoiseStd per-joint encoder position measurement-noise STD (rad) by joint name, for
    *                 the direct q update's R. Null lookup, or NaN / non-positive for a joint, falls back to
    *                 ENCODER_VAR for that joint. Wired variances are published as jointKF_encR_&lt;joint&gt;.
    * @param encoderVelocityNoiseStd per-joint encoder velocity measurement-noise STD (rad/s) by joint name.
    *                 Consumed (a) for base-&gt;foot chain joints that are not filter states (Alex: the ankles),
    *                 whose measured qd is folded into the stance-anchor measurement and its noise into the
    *                 anchor covariance by the rank-1 congruence J_U sigma^2 J_U^T, and (b) as the noise floor
    *                 of the direct-velocity channel for the filtered joints when that channel is enabled.
    *                 Null / NaN / non-positive falls back to SIGMA_QD_UNFILTERED for that joint.
    * @param jointVelocityMeasurementBreakFrequencyHz effective first-order corner (Hz) of the low-pass cascade
    *                 the measured joint velocity passed through before reaching this filter (drive-side LPF +
    *                 sensor-processing alpha: 1/f_eff = sum of 1/f_stage). Drives the direct-velocity channel's
    *                 adaptive lag inflation; null / NaN / non-positive disables inflation for that joint (the
    *                 measurement is treated as instantaneous — correct in sim, honest on hardware only when the
    *                 corner sits far above the motion band).
    * @param useDirectVelocityMeasurement boot-time default for the direct-velocity channel; the live YoBoolean
    *                 jointKFUseDirectVelocityMeasurement can flip it mid-run for hardware A/Bs.
    * @param sigmaTauOverride boot-time override (N*m) for {@code jointKFParam_sigmaTau} (the fallback
    *                 unmodeled-torque STD, {@link JointKFParameters#SIGMA_TAU}). NaN leaves the default. Exists
    *                 for offline NIS-based retuning sweeps (e.g. from a log-replay harness): sigmaTau is
    *                 consumed once at construction (see {@link JointKFPrediction}'s
    *                 {@code createRotorInertiaAndSigmaTauParameters}), so it cannot be swept by editing the live
    *                 YoDouble after this filter is built -- each candidate value needs its own instance.
    */
   JointLevelKFPreFilter(SensorOutputMapReadOnly sensorMap,
                         List<IMUBasedJointStateEstimatorParameters> pairParameters,
                         Collection<RigidBodyBasics> feet,
                         RigidBodyBasics rootBody,
                         ToDoubleFunction<String> encoderPositionNoiseStd,
                         ToDoubleFunction<String> encoderVelocityNoiseStd,
                         ToDoubleFunction<String> jointVelocityMeasurementBreakFrequencyHz,
                         boolean useDirectVelocityMeasurement,
                         double sigmaTauOverride,
                         double estimatorDT,
                         YoRegistry parentRegistry)
   {
      // Parameters first: every component below reads its tuning from here, and the LIVE ones keep being
      // re-read on the hot path afterwards. The override, if any, must land here -- before State/Prediction
      // are built below -- since Prediction reads sigmaTau exactly once, at construction.
      this.parameters = new JointKFParameters(registry);
      if (Double.isFinite(sigmaTauOverride))
         parameters.sigmaTau.set(sigmaTauOverride);
      this.requiredOnGroundTicks = Math.max(1, (int) Math.round(parameters.onGroundInitDebounce.getValue() / estimatorDT));
      if (parentRegistry != null)
         parentRegistry.addChild(registry);

      // ORDER IS LOAD-BEARING: State fixes the layout (n, m, dim, pairs, anchors, base IMU, maxStackRows) that
      // both Prediction and Update size themselves against, and BiasUpdate applies its block through Update's
      // Joseph core. All four share the one flat registry above, so simple names stay unique.
      this.state = new JointKFState(sensorMap, pairParameters, feet, encoderVelocityNoiseStd, estimatorDT, parameters, registry);
      this.prediction = new JointKFPrediction(state, rootBody, parameters, registry);
      this.update = new JointKFUpdate(state,
                                      encoderPositionNoiseStd,
                                      encoderVelocityNoiseStd,
                                      jointVelocityMeasurementBreakFrequencyHz,
                                      useDirectVelocityMeasurement,
                                      parameters,
                                      registry);
      this.biasUpdate = new JointKFBiasUpdate(state, update, parameters, registry);

      validateConstantModel();

      // Boot-time census (Part B item 6): every hardware boot prints the chain-DoF layout and, for each 1-DoF
      // chain, the joint axis and its two pure-bias sentinel rows (the min-side rows whose S floor is Sigma).
      state.logChainDoFCensus();
   }

   /**
    * The constant model matrices are built once and reused every tick, so a single non-finite entry — most
    * plausibly an IMU returning a non-finite bias process-noise covariance while still booting — permanently
    * poisons Q or F and makes P go NaN on the first predict, with no recovery. Naming the matrix at boot beats
    * discovering a silent all-NaN filter later.
    */
   private void validateConstantModel()
   {
      if (containsNonFinite(prediction.F))
         LogTools.error("JointLevelKFPreFilter transition matrix F is non-finite at construction.");
      if (containsNonFinite(prediction.Q))
         LogTools.error("JointLevelKFPreFilter process-noise Q is non-finite at construction — check the per-IMU "
                        + "angular-velocity bias process-noise covariances (see JointKFPrediction.buildProcessNoise).");
      if (containsNonFinite(update.Henc))
         LogTools.error("JointLevelKFPreFilter encoder Jacobian Henc is non-finite at construction.");
      if (containsNonFinite(update.Renc))
         LogTools.error("JointLevelKFPreFilter encoder noise Renc is non-finite at construction.");
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
      return createForKinematicsEstimator(sensorOutputMap,
                                          stateEstimatorParameters,
                                          imuProcessedOutputs,
                                          feet,
                                          estimatorRootBody,
                                          gravitationalAcceleration,
                                          cancelGravityFromAccelerationMeasurement,
                                          Double.NaN,
                                          estimatorDT,
                                          parentRegistry);
   }

   /** @param sigmaTauOverride see the 11-arg constructor's javadoc. NaN leaves the boot default. */
   public static JointLevelKFPreFilter createForKinematicsEstimator(SensorOutputMapReadOnly sensorOutputMap,
                                                                    StateEstimatorParameters stateEstimatorParameters,
                                                                    List<? extends IMUSensorReadOnly> imuProcessedOutputs,
                                                                    Collection<RigidBodyBasics> feet,
                                                                    RigidBodyBasics estimatorRootBody,
                                                                    double gravitationalAcceleration,
                                                                    BooleanProvider cancelGravityFromAccelerationMeasurement,
                                                                    double sigmaTauOverride,
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
                                       stateEstimatorParameters::getEncoderPositionMeasurementStandardDeviation,
                                       stateEstimatorParameters::getEncoderVelocityMeasurementStandardDeviation,
                                       stateEstimatorParameters::getJointVelocityMeasurementBreakFrequency,
                                       stateEstimatorParameters.useDirectJointVelocityMeasurementInJointKF(),
                                       sigmaTauOverride,
                                       estimatorDT,
                                       parentRegistry);
   }

   // ================================ Initialization ================================

   /**
    * Installs the optional on-ground gate: while {@code onGround} does not read true, {@link #initialize()}
    * stays deferred and consumers fall back to the raw sensors. Null disables gating (the default). Debounced
    * over {@code JointKFParameters.ON_GROUND_INIT_DEBOUNCE} — see that constant for why one tick is not enough.
    */
   public void setInitializationGate(BooleanSupplier onGround)
   {
      this.onGroundGate = onGround;
      this.consecutiveOnGroundTicks = 0;
      yoWaitingForGroundContact.set(onGround != null && !state.initialized);
   }

   /**
    * True when clear to seed: no gate wired, or the gate has read true for {@link #requiredOnGroundTicks}
    * consecutive attempts. Advances the debounce counter as a side effect, so call it exactly once per attempt.
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
      // Do not seed until the robot is firmly on the ground: while hanging, the exported base gyro bias is
      // unobservable (its only anchor is gated off with no trusted foot), so it wanders and the downstream InEKF
      // integrates that into a rotating base. Staying uninitialized exports NaN/zero, which consumers treat as
      // "fall back to the raw sensors". No-op when no gate is wired.
      if (!readyToInitialize())
         return;

      // Refuse to latch non-finite boot data (see JointKFState.seed): stay uninitialized and retry next tick.
      if (!state.seed())
         return;

      yoInitialized.set(true);
      yoWaitingForGroundContact.set(false);
   }

   // ================================ Phase 1 ================================

   @Override
   public void computeJointState()
   {
      // Phase 1: joint predict + encoder/pair-gyro measurement updates go here.
      if (!state.initialized)
      {
         initialize();
         if (!state.initialized)
            return; // boot data not valid yet; consumers keep falling back to raw sensors until it is
      }

      prediction.predict();
      state.warnIfNonFiniteState("predict", -1);

      // Encoder update, gated PER JOINT (not all-or-nothing). Alex's encoders are intermittent, so a single
      // non-finite reading used to drop the WHOLE encoder update — unpinning every joint's position for that
      // tick. With the position pin gone, q integrates q̇ (which the pair-gyro/stance-anchor updates only
      // loosely constrain, and the anchor can bias) with no correction, so q ramps without bound and, on the
      // mass-matrix path, P grows without bound too — the divergence that drove the joint estimates to ~1e18.
      // Fix: build the encoder measurement over only the joints whose encoder is finite this tick, so every
      // good encoder keeps pinning its joint while just the bad one is skipped. When all are finite (the common
      // case) this is exactly the previous full-rank update; the reduced matrices reuse the pre-sized scratch
      // (reshape to a smaller size never allocates), so it stays allocation-free on the estimator thread.
      int validEncoderCount = update.buildValidEncoderMeasurement();
      if (validEncoderCount > 0)
      {
         update.josephUpdate(update.Henc, update.zEnc, update.Renc, JointKFUpdate.Channel.ENCODER);
         state.warnIfNonFiniteState("encoderUpdate", -1);
      }

      // Direct-velocity update: firmware-reported joint velocity as a measurement of q̇, with the per-tick
      // adaptive lag inflation on R. Skipped wholesale on any non-finite velocity, like the encoder block.
      if (update.isDirectVelocityEnabled())
      {
         // Same identity as the encoder block above: z_qd row i is state index i, against Hqd = [0 | I_n | 0]
         // and an Rqd diagonal keyed by state index. prevZqd[] and qdSlewSmoothed[] are state-index-keyed too.
         boolean velocitiesValid = true;
         for (int i = 0; i < state.numberOfJoints; i++)
         {
            OneDoFJointBasics j = state.jointsByIndex[i];
            double qd = state.sensorMap.getOneDoFJointOutput(j).getVelocity();
            if (!Double.isFinite(qd))
            {
               velocitiesValid = false;
               if (!state.warnedNonFiniteInput)
                  state.warnNonFiniteInputOnce("joint velocity of " + j.getName());
            }
            update.zqd.set(i, 0, qd);
         }
         if (velocitiesValid)
         {
            // Inside the validity guard on purpose: the slew state must NOT advance on an invalid tick.
            update.refreshDirectVelocityNoise();
            update.josephUpdate(update.Hqd, update.zqd, update.Rqd, JointKFUpdate.Channel.ENCODER_VELOCITY);
            state.warnIfNonFiniteState("encoderVelocityUpdate", -1);
         }
      }

      // All pair rows plus the active stance-anchor rows in ONE Joseph update, using the previous tick's
      // trusted-feet set (cached in computeImuBiases).
      biasUpdate.applyStackedUpdate();

      // The Joseph products keep P symmetric only to round-off, and a slow asymmetry drift eventually corrupts
      // the Cholesky conditioning gate. O(dim^2), negligible next to the O(dim^3) products. Runs BEFORE the Yo
      // publication so the confidence envelope reads the symmetrized P.
      symmetrize(state.P);

      state.updateJointYoVariables();
      biasUpdate.updateBiasYoVariables();
   }

   // ================================ Phase 2 ================================

   @Override
   public void computeImuBiases(List<RigidBodyBasics> trustedFeet)
   {
      // Bookkeeping only: the stance anchors moved into the phase-1 stacked update, so this just caches the set.
      biasUpdate.cacheTrustedFeet(trustedFeet);
   }

   // ================================ Shared matrix helpers ================================
   // Static, stateless EJML helpers shared by the components. They live here beside setMatrix, which main-source
   // code outside this package already calls (invariantEstimator/InvariantState).

   public static void setMatrix(DMatrixRMaj out, RotationMatrixReadOnly r)
   {
      r.get(out);
   }

   /** In-place symmetrization A <- 0.5 (A + A^T). Allocation-free. */
   static void symmetrize(DMatrixRMaj a)
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

   /** True if any entry of the matrix is NaN or infinite. Allocation-free; O(elements). */
   static boolean containsNonFinite(DMatrixRMaj mat)
   {
      for (int i = 0; i < mat.getNumElements(); i++)
      {
         if (!Double.isFinite(mat.get(i)))
            return true;
      }
      return false;
   }

   // ================================ Consumed state ================================

   @Override
   public boolean containsJoint(OneDoFJointBasics joint)
   {
      return state.isFilteredJoint(joint);
   }

   @Override
   public double getEstimatedJointPosition(OneDoFJointBasics joint)
   {
      int stateIndex = state.jointIndex(joint);
      return (stateIndex == JointKFState.NOT_IN_STATE || !state.initialized) ? Double.NaN : state.x.get(stateIndex);
   }

   @Override
   public double getEstimatedJointVelocity(OneDoFJointBasics joint)
   {
      int stateIndex = state.jointIndex(joint);
      return (stateIndex == JointKFState.NOT_IN_STATE || !state.initialized) ? Double.NaN : state.x.get(state.numberOfJoints + stateIndex);
   }

   @Override
   public boolean hasCovariance()
   {
      return state.initialized;
   }

   @Override
   public void packPositionCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack)
   {
      packCov(joints, 0, fallbackVariance, toPack);
   }

   @Override
   public void packVelocityCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack)
   {
      packCov(joints, state.numberOfJoints, fallbackVariance, toPack);
   }

   private void packCov(OneDoFJointBasics[] joints, int blockOffset, double fallbackVariance, DMatrixRMaj toPack)
   {
      int numberOfChainJoints = joints.length;
      toPack.reshape(numberOfChainJoints, numberOfChainJoints);
      toPack.zero();
      for (int a = 0; a < numberOfChainJoints; a++)
      {
         int indexOfJointA = state.jointIndex(joints[a]);
         if (indexOfJointA == JointKFState.NOT_IN_STATE)
         {
            toPack.set(a, a, fallbackVariance);
            continue;
         }
         for (int b = 0; b < numberOfChainJoints; b++)
         {
            int indexOfJointB = state.jointIndex(joints[b]);
            if (indexOfJointB != JointKFState.NOT_IN_STATE)
               toPack.set(a, b, state.P.get(blockOffset + indexOfJointA, blockOffset + indexOfJointB)); // adding the off diagonals for the cross-covariances
         }
      }
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      int imuOrdinal = state.imuOrdinal(imu);
      if (imuOrdinal == JointKFState.NOT_IN_STATE || !state.initialized)
      {
         // IMU not in the filter's state (e.g. the primary pelvis IMU when it isn't a pair member), or the
         // filter hasn't initialized yet: report zero bias. MUST return here, and the reason is now STRONGER
         // than it was: this used to guard an NPE on unboxing a null imuOrdinal in "3 * imuOrdinal". With the -1 sentinel
         // the fall-through is worse than a crash — 2n + 3*(-1) = 2n-3 is a VALID index into x, so it would
         // silently export three JOINT VELOCITIES as this IMU's gyro bias, straight into the InEKF's predict.
         // No exception, no NaN, just a wrong robot.
         biasOut.setToZero(imu.getMeasurementFrame());
         return biasOut;
      }
      int col = 2 * state.numberOfJoints + 3 * imuOrdinal;
      double bx = state.x.get(col);
      double by = state.x.get(col + 1);
      double bz = state.x.get(col + 2);
      if (!Double.isFinite(bx) || !Double.isFinite(by) || !Double.isFinite(bz))
      {
         // Fail soft like the alpha filter's bias provider: never export a non-finite bias. This is
         // consumed directly by the InEKF's predict (gyro - bias -> R*exp(phi*dt)), which throws
         // NotARotationMatrixException on NaN — a hardware-only crash the joint NaN-fallback can't catch.
         state.warnNonFiniteInputOnce("bias state of " + imu.getSensorName());
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

   int getStateDimension()          { return state.dim; }
   int getNumberOfFilteredJoints()  { return state.numberOfJoints; }
   int getNumberOfIMUs()            { return state.numberOfIMUs; }
   int getNumberOfPairs()           { return state.pairs.size(); }

   /** State index of the given joint's position entry (its velocity entry is this + n); -1 if not filtered. */
   int getJointStateIndex(OneDoFJointBasics joint) { return state.jointIndex(joint); }
   /** Immutable snapshot: jointsByIndex IS the filter's state order and must not escape as a mutable array. */
   List<OneDoFJointBasics> getFilteredJointsInStateOrder() { return List.of(state.jointsByIndex); }

   /** This tick's encoder measurement vector; row i must be the joint at state index i. */
   DMatrixRMaj getEncoderMeasurementForTest()        { return update.zEnc.copy(); }
   /** This tick's direct-velocity measurement vector; row i must be the joint at state index i. */
   DMatrixRMaj getDirectVelocityMeasurementForTest() { return update.zqd.copy(); }

   DMatrixRMaj getStateVector()      { return state.x.copy(); }     // x = [q ; q_dot ; b_omega]
   DMatrixRMaj getCovariance()       { return state.P.copy(); }
   DMatrixRMaj getTransitionMatrix() { return prediction.F.copy(); }
   DMatrixRMaj getProcessNoise()     { return prediction.Q.copy(); }

   /** True when Q's joint blocks come from Qa = sigma_tau^2 M(q)^-2 (a robot model was provided). */
   boolean isUsingMassMatrixProcessNoise() { return prediction.isUsingMassMatrixProcessNoise(); }
   /** Recomputes the mass-matrix Q blocks from the model's current configuration (no-op on the scalar path). */
   void updateProcessNoiseFromMassMatrixForTest() { prediction.updateProcessNoiseFromMassMatrix(); }
   /** EKF time update in isolation: x <- F x, P <- F P F^T + Q(q). */
   void predict() { prediction.predict(); }

   DMatrixRMaj getEncoderJacobian()  { return update.Henc.copy(); }
   DMatrixRMaj getEncoderNoise()     { return update.Renc.copy(); }
   DMatrixRMaj getVelocityMeasurementJacobian() { return update.Hqd.copy(); }
   /** This tick's APPLIED velocity R (floor + lag inflation); call refreshDirectVelocityNoise()/update() first. */
   DMatrixRMaj getVelocityMeasurementNoise()    { return update.Rqd.copy(); }
   /** Loads a measured q̇ vector (filter state order) into zqd so tests can drive refreshDirectVelocityNoise(). */
   void setDirectVelocityMeasurementForTest(double[] qdMeasured) { update.setDirectVelocityMeasurementForTest(qdMeasured); }
   void refreshDirectVelocityNoise() { update.refreshDirectVelocityNoise(); }

   void josephUpdate(DMatrixRMaj Hm, DMatrixRMaj zm, DMatrixRMaj Rm) { update.josephUpdate(Hm, zm, Rm); }
   void josephUpdate(DMatrixRMaj Hm, DMatrixRMaj zm, DMatrixRMaj Rm, String label) { update.josephUpdate(Hm, zm, Rm, label); }
   String describeSingularInnovation(String label, String reason, DMatrixRMaj Hm, DMatrixRMaj Rm)
   {
      return update.describeSingularInnovation(label, reason, Hm, Rm);
   }

   /** Overwrites the mean and covariance (and marks initialized) so tests can drive predict()/josephUpdate() from a known prior. */
   void setStateForTest(DMatrixRMaj xPrior, DMatrixRMaj pPrior)
   {
      state.x.set(xPrior);
      state.P.set(pPrior);
      state.initialized = true;
   }

   /** Builds the stacked measurement without applying it. Set the anchors first with
    *  {@link #setTrustedFeetForTest} (empty => pairs-only, K = 0). */
   void buildStackedMeasurementForTest() { biasUpdate.buildStackedMeasurement(); }
   DMatrixRMaj getStackedMeasurementJacobian() { return biasUpdate.Hg.copy(); }  // H_g (3(E+K) x dim), reshaped to this tick
   DMatrixRMaj getStackedMeasurementResidual() { return biasUpdate.zg.copy(); }  // z_g (3(E+K) x 1)
   DMatrixRMaj getStackedMeasurementNoise()    { return biasUpdate.Rg.copy(); }  // R_g (3(E+K) x 3(E+K))
   DMatrixRMaj getMixingOperator()             { return biasUpdate.Lmix.copy(); } // L (3(E+K) x 3m); H_g bias columns == this
   int getStackedRowForPair(int pairIndex)     { return 3 * pairIndex; }      // pair e occupies rows [3e, 3e+3)
   int getBiasBlockColumn(IMUSensorReadOnly imu) { return 2 * state.numberOfJoints + 3 * state.requireImuOrdinal(imu); } // state col of imu's bias
   int getImuOrdinal(IMUSensorReadOnly imu)    { return state.requireImuOrdinal(imu); }
   /** The ORDINAL-INDEXED array side, so a test can check it against the map the way it does for joints. */
   IMUSensorReadOnly getImuByOrdinal(int ordinal) { return state.imusByOrdinal[ordinal]; }
   IMUSensorReadOnly getBaseIMU()              { return state.baseIMU; }
   /** Anchors active on the last {@link #buildStackedMeasurementForTest}. 0 => the base-bias gauge is unfixed. */
   int getActiveAnchorCountForTest()           { return biasUpdate.getActiveAnchorCount(); }

   /** Overwrites the cached previous-tick trusted-feet set that the stacked build reads anchors from. */
   void setTrustedFeetForTest(List<RigidBodyBasics> feet) { biasUpdate.setTrustedFeetForTest(feet); }

   int[] getPairVelocityColumns(int pairIndex) { return state.pairs.get(pairIndex).qdCols.clone(); }
   int getPairParentBiasColumn(int pairIndex)  { return state.pairs.get(pairIndex).parentBias; }
   int getPairChildBiasColumn(int pairIndex)   { return state.pairs.get(pairIndex).childBias; }
}

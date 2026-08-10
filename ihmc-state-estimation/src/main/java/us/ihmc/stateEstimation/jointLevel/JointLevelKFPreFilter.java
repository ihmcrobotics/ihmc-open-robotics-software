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
 * <p><b>This class is the orchestrator.</b> It owns the tick sequence, the on-ground initialization gate and the
 * {@code OneDoFJointStateSource} / {@code IMUBiasProvider} surface the estimator consumes; the filter itself is
 * five collaborators, each with one job:</p>
 * <ul>
 *   <li>{@link JointKFParameters} — every tuning constant, published as YoVariables so the documented
 *   calibration loops run live in SCS instead of through a recompile;</li>
 *   <li>{@link JointKFState} — the state layout (which joints and IMUs, in what order), x and P, and the
 *   {@code Pair} / {@code FootAnchor} structural holders;</li>
 *   <li>{@link JointKFPrediction} — F, Q, and the Schur-complement mass-matrix process noise;</li>
 *   <li>{@link JointKFUpdate} — the encoder and direct-velocity channels plus the shared Joseph core and its
 *   conditioning gates;</li>
 *   <li>{@link JointKFBiasUpdate} — the stacked gyro measurement (pair rows + stance anchors), the only channel
 *   that observes the per-IMU gyro biases.</li>
 * </ul>
 * They are constructed in that order and depend on each other one way only
 * (BiasUpdate → Update → State), so there are no initialization cycles.
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

   // Observability: the constructor was previously handed a parentRegistry that it dropped on the floor
   // (so the filter published nothing on hardware). These are wired to the parent registry now. ONE flat
   // registry is shared by every component, so YoVariable simple names stay unique and their full namespaces
   // are unchanged by the split.
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoBoolean yoInitialized = new YoBoolean("jointKFInitialized", registry);
   private final YoBoolean yoWaitingForGroundContact = new YoBoolean("jointKFWaitingForGroundContact", registry);

   private final FrameVector3D biasOut = new FrameVector3D();

   // Package-private (not private) so the allocation/behavior tests in this package can build it directly
   // from a synthetic IMU-pair setup without standing up a full StateEstimatorParameters.
   /** Scalar-CWNA process-noise overload: no robot model, so Qa = SIGMA_ACCEL^2 I (the pre-mass-matrix behavior). */
   JointLevelKFPreFilter(SensorOutputMapReadOnly sensorMap,
                         List<IMUBasedJointStateEstimatorParameters> pairParameters,
                         Collection<RigidBodyBasics> feet,
                         double estimatorDT,
                         YoRegistry parentRegistry)
   {
      this(sensorMap, pairParameters, feet, null, null, null, null, false, estimatorDT, parentRegistry);
   }

   /** Overload without per-joint encoder noise lookups: every joint uses the scalar fallbacks. */
   JointLevelKFPreFilter(SensorOutputMapReadOnly sensorMap,
                         List<IMUBasedJointStateEstimatorParameters> pairParameters,
                         Collection<RigidBodyBasics> feet,
                         RigidBodyBasics rootBody,
                         double estimatorDT,
                         YoRegistry parentRegistry)
   {
      this(sensorMap, pairParameters, feet, rootBody, null, null, null, false, estimatorDT, parentRegistry);
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
      this(sensorMap, pairParameters, feet, rootBody, encoderPositionNoiseStd, encoderVelocityNoiseStd, null, false, estimatorDT, parentRegistry);
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
    */
   JointLevelKFPreFilter(SensorOutputMapReadOnly sensorMap,
                         List<IMUBasedJointStateEstimatorParameters> pairParameters,
                         Collection<RigidBodyBasics> feet,
                         RigidBodyBasics rootBody,
                         ToDoubleFunction<String> encoderPositionNoiseStd,
                         ToDoubleFunction<String> encoderVelocityNoiseStd,
                         ToDoubleFunction<String> jointVelocityMeasurementBreakFrequencyHz,
                         boolean useDirectVelocityMeasurement,
                         double estimatorDT,
                         YoRegistry parentRegistry)
   {
      // Parameters first: every component below reads its tuning from here, and the LIVE ones keep being
      // re-read on the hot path afterwards.
      this.parameters = new JointKFParameters(registry);
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
    * One-time (construction) finiteness check on the constant model matrices. These are built once and reused
    * every tick, so a single non-finite entry here — most plausibly an IMU that returns a non-finite
    * angular-velocity bias process-noise covariance while it is still booting — permanently poisons Q (or F)
    * and makes P go NaN on the first predict, with no recovery. Logged as an error naming the matrix so this
    * shows up clearly at estimator start instead of as a silent all-NaN filter. Runs at construction only; the
    * string work here is not on the estimator hot path.
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
                                       estimatorDT,
                                       parentRegistry);
   }

   // ================================ Initialization ================================

   /**
    * Installs the optional on-ground gate: while {@code onGround} does not read true, {@link #initialize()}
    * stays deferred (the filter exports NaN joint states and zero bias, so consumers fall back to the raw
    * sensors). Pass {@code null} to disable gating (the default — initialize as soon as boot data is finite).
    * The gate is debounced internally over {@code JointKFParameters.ON_GROUND_INIT_DEBOUNCE}; see that constant
    * for why a single-tick check is not enough. Called once at wiring time by the consumer that owns the
    * contact-probability signal; never on the estimator hot path.
    */
   public void setInitializationGate(BooleanSupplier onGround)
   {
      this.onGroundGate = onGround;
      this.consecutiveOnGroundTicks = 0;
      yoWaitingForGroundContact.set(onGround != null && !state.initialized);
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
      // unobservable while hanging (its only anchor is the stance-anchor row, gated off when no foot is
      // trusted), so seeding then lets the bias wander and the downstream InEKF integrates that into a
      // rotating/glitching base. Staying uninitialized exports NaN/zero, which the consumers treat as
      // "fall back to the raw gyro/sensors". No-op when no gate is wired (unit tests / non-invariant pipelines).
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

      // update encoder states; skipped wholesale if any encoder reads non-finite (boot transient).
      // The z fills stay INLINE here rather than moving behind a component call: they iterate
      // jointToIndex.keySet(), whose iterator only fits the allocation test's 32 B/tick budget because C2
      // scalar-replaces it, and that elimination depends on the allocation, the loop and the consumer inlining
      // into one compilation unit. They are orchestration anyway — they drive encodersValid and the warner.
      int row = 0;
      boolean encodersValid = true;
      for (OneDoFJointBasics j : state.jointToIndex.keySet())
      {
         double q = state.sensorMap.getOneDoFJointOutput(j).getPosition();
         if (!Double.isFinite(q))
         {
            encodersValid = false;
            if (!state.warnedNonFiniteInput)
               state.warnNonFiniteInputOnce("joint position of " + j.getName());
         }
         update.zEnc.set(row++, 0, q);
      }
      if (encodersValid)
      {
         update.josephUpdate(update.Henc, update.zEnc, update.Renc, JointKFUpdate.Channel.ENCODER);
         state.warnIfNonFiniteState("encoderUpdate", -1);
      }

      // Direct-velocity update: the firmware-reported joint velocity as a measurement of the q̇ states, with
      // the per-tick adaptive lag inflation on R (see refreshDirectVelocityNoise). Skipped wholesale on any
      // non-finite velocity, like the encoder block.
      if (update.isDirectVelocityEnabled())
      {
         int qdRow = 0;
         boolean velocitiesValid = true;
         for (OneDoFJointBasics j : state.jointToIndex.keySet())
         {
            double qd = state.sensorMap.getOneDoFJointOutput(j).getVelocity();
            if (!Double.isFinite(qd))
            {
               velocitiesValid = false;
               if (!state.warnedNonFiniteInput)
                  state.warnNonFiniteInputOnce("joint velocity of " + j.getName());
            }
            update.zqd.set(qdRow++, 0, qd);
         }
         if (velocitiesValid)
         {
            // Inside the validity guard on purpose: the slew state must NOT advance on an invalid tick.
            update.refreshDirectVelocityNoise();
            update.josephUpdate(update.Hqd, update.zqd, update.Rqd, JointKFUpdate.Channel.ENCODER_VELOCITY);
            state.warnIfNonFiniteState("encoderVelocityUpdate", -1);
         }
      }

      // Single stacked gyro update (SPEC §5-§6): all pair rows plus the active stance-anchor rows in ONE Joseph
      // update, using the previous tick's trusted-feet set (cached in computeImuBiases).
      biasUpdate.applyStackedUpdate();

      // P hygiene (Part B item 7): re-symmetrize P once per tick. The Joseph products keep P symmetric only to
      // round-off; a slow asymmetry drift eventually corrupts the Cholesky conditioning gate. O(dim^2),
      // negligible next to the Joseph O(dim^3) products. Runs BEFORE the Yo publication so the confidence
      // envelope reads the symmetrized P.
      symmetrize(state.P);

      state.updateJointYoVariables();
      biasUpdate.updateBiasYoVariables();
   }

   // ================================ Phase 2 ================================

   @Override
   public void computeImuBiases(List<RigidBodyBasics> trustedFeet)
   {
      // Phase 2 is now bookkeeping only (SPEC §6 phase note): the stance anchors have moved into the phase-1
      // stacked update, so all this does is cache THIS tick's trusted-feet set for next tick's anchor rows.
      biasUpdate.cacheTrustedFeet(trustedFeet);
   }

   // ================================ Shared matrix helpers ================================
   // Static, stateless EJML helpers shared by the components. They live here beside set_matrix, which main-source
   // code outside this package already calls (invariantEstimator/InvariantState).

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
      return state.jointToIndex.containsKey(joint);
   }

   @Override
   public double getEstimatedJointPosition(OneDoFJointBasics joint)
   {
      Integer idx = state.jointToIndex.get(joint);
      return (idx == null || !state.initialized) ? Double.NaN : state.x.get(idx);
   }

   @Override
   public double getEstimatedJointVelocity(OneDoFJointBasics joint)
   {
      Integer idx = state.jointToIndex.get(joint);
      return (idx == null || !state.initialized) ? Double.NaN : state.x.get(state.n + idx);
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
      packCov(joints, state.n, fallbackVariance, toPack);
   }

   private void packCov(OneDoFJointBasics[] joints, int blockOffset, double fallbackVariance, DMatrixRMaj toPack)
   {
      int mm = joints.length;
      toPack.reshape(mm, mm);
      toPack.zero();
      for (int a = 0; a < mm; a++)
      {
         Integer ia = state.jointToIndex.get(joints[a]);
         if (ia == null)
         {
            toPack.set(a, a, fallbackVariance);
            continue;
         }
         for (int b = 0; b < mm; b++)
         {
            Integer ib = state.jointToIndex.get(joints[b]);
            if (ib != null)
               toPack.set(a, b, state.P.get(blockOffset + ia, blockOffset + ib)); // adding the off diagonals for the cross-covariances
         }
      }
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      Integer ord = state.imuToOrdinal.get(imu);
      if (ord == null || !state.initialized)
      {
         // IMU not in the filter's state (e.g. the primary pelvis IMU when it isn't a pair member),
         // or the filter hasn't initialized yet: report zero bias. MUST return here — falling through
         // would unbox a null ord in "3 * ord" and NPE every tick on the estimator thread.
         biasOut.setToZero(imu.getMeasurementFrame());
         return biasOut;
      }
      int col = 2 * state.n + 3 * ord;
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
   int getNumberOfFilteredJoints()  { return state.n; }
   int getNumberOfIMUs()            { return state.m; }
   int getNumberOfPairs()           { return state.pairs.size(); }

   /** State index of the given joint's position entry (its velocity entry is this + n); -1 if not filtered. */
   int getJointStateIndex(OneDoFJointBasics joint) { Integer i = state.jointToIndex.get(joint); return i == null ? -1 : i; }
   List<OneDoFJointBasics> getFilteredJointsInStateOrder() { return new ArrayList<>(state.jointToIndex.keySet()); }

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

   /**
    * Builds the stacked gyro measurement (H_g, z_g, R_g, L) from the current model configuration and the cached
    * trusted-feet-from-last-tick set, WITHOUT applying the update, for inspection. Set the active anchors first
    * with {@link #setTrustedFeetForTest} (empty => pairs-only, K = 0).
    */
   void buildStackedMeasurementForTest() { biasUpdate.buildStackedMeasurement(); }
   DMatrixRMaj getStackedMeasurementJacobian() { return biasUpdate.Hg.copy(); }  // H_g (3(E+K) x dim), reshaped to this tick
   DMatrixRMaj getStackedMeasurementResidual() { return biasUpdate.zg.copy(); }  // z_g (3(E+K) x 1)
   DMatrixRMaj getStackedMeasurementNoise()    { return biasUpdate.Rg.copy(); }  // R_g (3(E+K) x 3(E+K))
   DMatrixRMaj getMixingOperator()             { return biasUpdate.Lmix.copy(); } // L (3(E+K) x 3m); H_g bias columns == this
   int getStackedRowForPair(int pairIndex)     { return 3 * pairIndex; }      // pair e occupies rows [3e, 3e+3)
   int getBiasBlockColumn(IMUSensorReadOnly imu) { return 2 * state.n + 3 * state.imuToOrdinal.get(imu); } // state col of imu's bias
   int getImuOrdinal(IMUSensorReadOnly imu)    { return state.imuToOrdinal.get(imu); }
   IMUSensorReadOnly getBaseIMU()              { return state.baseIMU; }
   /** Anchors active on the last {@link #buildStackedMeasurementForTest}. 0 => the base-bias gauge is unfixed. */
   int getActiveAnchorCountForTest()           { return biasUpdate.getActiveAnchorCount(); }

   /** Overwrites the cached previous-tick trusted-feet set that the stacked build reads anchors from. */
   void setTrustedFeetForTest(List<RigidBodyBasics> feet) { biasUpdate.setTrustedFeetForTest(feet); }

   int[] getPairVelocityColumns(int pairIndex) { return state.pairs.get(pairIndex).qdCols.clone(); }
   int getPairParentBiasColumn(int pairIndex)  { return state.pairs.get(pairIndex).parentBias; }
   int getPairChildBiasColumn(int pairIndex)   { return state.pairs.get(pairIndex).childBias; }
}

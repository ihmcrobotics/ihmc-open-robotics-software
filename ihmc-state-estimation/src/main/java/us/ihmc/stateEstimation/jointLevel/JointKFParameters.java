package us.ihmc.stateEstimation.jointLevel;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Every tuning constant of the joint-level KF ({@link JointLevelKFPreFilter} and its components), published as
 * YoVariables so the calibration procedures documented on the constants below can be run live in SCS instead of
 * through a recompile-reflash-rerun cycle.
 *
 * <p><b>One instance per filter, never static.</b> A YoVariable needs a registry, so a static holder would
 * collide on the second filter built in one JVM (YoRegistry rejects duplicate names) and would contradict the
 * filter's own "do not share an instance between pipelines" rule. The default TABLES ({@code ALPHA_*},
 * {@code ROTOR_INERTIA_*}) and their lookups are static — they are compile-time defaults that seed the
 * per-joint YoVariables, and they are what the package tests assert against.</p>
 *
 * <p><b>Two tiers, and the distinction is load-bearing.</b> Every parameter is published, but only some are
 * actually re-read while the filter runs. Each YoVariable's description states which it is, so SCS shows it on
 * hover:</p>
 * <ul>
 *   <li><b>LIVE</b> — read on the hot path every tick, so editing retunes the filter immediately:
 *   {@link #qaMax}, {@link #condSMax}, {@link #anchorVar}, {@link #sigmaGyroFloor}, and the per-joint
 *   {@code jointKFParam_alpha_<joint>} / {@code jointKFParam_rotorInertia_<joint>} published alongside the
 *   mass-matrix process noise.</li>
 *   <li><b>BOOT-TIME</b> — consumed once at construction (or, for the {@code init*Var} priors, when the filter
 *   seeds). Published for visibility and so a hardware log records the tuning that produced it; editing
 *   mid-run does nothing.</li>
 * </ul>
 *
 * <p>There are deliberately NO change listeners: a listener firing on the SCS GUI thread would mutate Q / R
 * underneath the estimator thread. The boot-time parameters are boot-time by nature and are not worth that
 * hazard.</p>
 *
 * @author Lucas Libshutz
 */
final class JointKFParameters
{
   // ================================ Defaults ================================
   // These are the values the filter shipped with; each YoVariable below is seeded from the matching constant.

   /**
    * Fallback encoder position variance (rad^2), used for any joint the per-joint lookup does not cover
    * (encoderPositionNoiseStd == null or returns NaN). sigma ~ 7.1e-3 rad. Hardware-measured per-joint values
    * (walking-run FFT/PSD noise floor, 2026-07-15) run sigma 5.6e-5..7.5e-4 rad — variances 2-4 ORDERS OF
    * MAGNITUDE below this fallback — so an Alex joint silently on the fallback badly under-trusts its encoder;
    * watch jointKF_encR_&lt;joint&gt; at boot.
    */
   static final double ENCODER_VAR = 5.0e-5;
   /** rad/s^2 CWNA process-noise STD (scalar fallback when no robot model is provided). */
   static final double SIGMA_ACCEL = 50.0;
   /**
    * N*m unmodeled-torque STD for the mass-matrix path: Qa = Lambda_eff^-1 diag(sigma_tau,i^2) Lambda_eff^-T.
    * SIGMA_TAU is now only the FALLBACK STD used for a joint whose effort limit is absent/non-finite; the live
    * per-joint STD is sigma_tau,i = ALPHA * tau_max,i (see ALPHA and the per-joint alpha YoVariables).
    * Kept at 5 N*m as the Rev.2 interim (Lambda^-2 ⪰ M_jj^-2 inflated Qa vs. the Rev.1 locked-base map).
    * TODO(retune) with ALPHA.
    */
   static final double SIGMA_TAU = 5.0;

   // Per-joint unmodeled-torque STD as a fraction of the joint's effort limit: sigma_tau,i = alpha_i * tau_max,i
   // (Part B item 3), tau_max,i from OneDoFJointReadOnly.getEffortLimitUpper(). Two levels of per-joint scaling:
   // tau_max,i already scales by each actuator's torque capacity (a hip 217 N*m vs a wrist a few), and alpha_i is
   // the dimensionless "fraction of capacity that is unmodeled". alpha_i is a scalar default with per-joint
   // OVERRIDES (below), matched by case-insensitive name substring exactly like the rotor table — so a joint the
   // QA_MAX tripwire flags can be knocked down without touching the rest of the robot.
   //
   // RETUNE PRINCIPLE (2026-07-10, hardware log 20260710_135507): a uniform ALPHA fixes the unmodeled torque at a
   // fixed FRACTION OF TORQUE CAPACITY, but the torque->acceleration map Lambda_eff^-1 varies by orders of
   // magnitude across joints, so sqrt(diag(Qa)_i) = |Lambda_eff^-1|_ii * alpha_i * tau_max,i spans orders of
   // magnitude and joints trip QA_MAX one after another (knee was knocked to 0.03; a hip/spine_Z is the new argmax
   // binding EVERY tick in that log). The physically-motivated fix is to equalize the unmodeled-ACCELERATION STD
   // (the CWNA quantity for a (q,qd) filter) across joints at a common target TARGET_QDD_STD:
   //     alpha_i = TARGET_QDD_STD / (|Lambda_eff^-1|_ii * tau_max,i)   ==>   alpha_i = alpha_old_i * sqrt(TARGET_QDD_STD^2 / diag(Qa)_i)
   // The second form is how to CALIBRATE from a run: read the per-joint yoQaDiag (jointKF_QaDiag_<joint>) at the
   // current alpha and rescale (iterate 2-3x; off-diagonal Lambda_eff^-1 coupling makes it not one-shot).
   // TARGET_QDD_STD = 20 rad/s^2 (var 400) gives a 2.25x variance margin under QA_MAX = 900, so quiet standing
   // sits below the cap and only genuine walking transients trip it (a tripwire, not a per-tick clamp).
   //
   // With the per-joint alpha now published as jointKFParam_alpha_<joint> (LIVE), that calibration loop runs in
   // SCS against jointKF_QaDiag_<joint> without a rebuild — which is the whole point of this class.
   //
   // CALIBRATED 2026-07-10 from a live STAND_PREP read of jointKF_QaDiag_<joint> on the instrumented build:
   // ALPHA_VALUES below are the equalized set alpha_i = alpha_old_i * sqrt(400 / diag(Qa)_i), one per filtered
   // joint. Cross-check: the LEFT/RIGHT pairs agree to ~0.2% (HIP_X 0.0874 vs 0.0875, HIP_Z/HIP_Y/KNEE likewise)
   // — the legs are physically identical, so that symmetry validates the measurement. The knee RAISED from the
   // old reactive 0.03 to ~0.071: in quiet STAND_PREP its diag(Qa) ~ 72 = (8.5 rad/s^2)^2 sat well UNDER target,
   // i.e. it was over-damped there (0.03 had been set against a more dynamic config that logged diag(Qa)=14842 at
   // alpha=0.15). diag(Qa) is CONFIGURATION-DEPENDENT, so re-read jointKF_QaDiag_<joint> after a gait change and
   // iterate until every sqrt(diag(Qa)) ~ 20 and the jointKF_QaCapBind_<joint>_count counters stay flat. All 9
   // currently-filtered joints are listed; ALPHA_DEFAULT is the fallback for any future unlisted filtered joint.

   /** rad/s^2, common unmodeled-acceleration STD target for the ALPHA equalization. */
   static final double TARGET_QDD_STD = 20.0;
   /** Fallback only (surfaces an unlisted filtered joint via the QA_MAX tripwire). */
   static final double ALPHA_DEFAULT = 0.15;
   private static final String[] ALPHA_JOINT_KEYS = {
         "SPINE_Z",
         "LEFT_HIP_X",  "LEFT_HIP_Z",  "LEFT_HIP_Y",  "LEFT_KNEE_Y",
         "RIGHT_HIP_X", "RIGHT_HIP_Z", "RIGHT_HIP_Y", "RIGHT_KNEE_Y"};
   private static final double[] ALPHA_VALUES = {
         5.61133e-2,
         6.01544e-2, 2.94985e-2, 2.82043e-2, 2.47925e-2,
         5.52493e-2, 3.34146e-2, 2.56966e-2, 2.49771e-2
   };

   /**
    * TRIPWIRE ONLY (Part B item 2 — no longer a scaler). Physical ceiling on the per-joint acceleration
    * process-noise VARIANCE (~ (30 rad/s^2)^2). With the reflected-rotor-inertia floor on Lambda_eff (item 1),
    * max diag(Qa) must sit far below this; if it ever would not, that is a model/config regression to SURFACE,
    * not to hide. When max diag(Qa) &gt; QA_MAX we warn once (naming the argmax joint) and count it in
    * jointKFQaCapWouldBindCount, but we DO NOT rescale — the old uniform CommonOps_DDRM.scale coupled one
    * joint's outlier into GLOBAL Q starvation (hips down ~6 orders), which collapsed P onto the measurement
    * floors and CAUSED the min-side S singularity. A cap that silently rescales the whole robot is worse than
    * the disease.
    */
   static final double QA_MAX = 900.0;

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
         //TODO: needs to be moved to use the calibrated inertia values either from alex-sdk or alex-hardware
         0.062, 0.02, 0.167, 0.167, 0.07, 0.05, 0.062,
         0.067, 0.067, 0.022, 0.022,
         0.005, 0.005, 0.005, 0.005, 0.005};
   /** Conservative floor for an unmatched filtered joint. */
   static final double ROTOR_INERTIA_DEFAULT = 0.005;

   /**
    * Gyro measurement-noise floor (Part B item 4). getAngularVelocityNoiseCovariance is zero on hardware whenever
    * the IMU's SensorNoiseParameters are unset (Alex historically ran with a null SensorNoiseParameters =&gt; all
    * covariances 0). A zero Sigma removes the innovation-covariance floor on the pure-bias rows of every 1-DoF
    * chain, collapsing lambda_min(S) (the logged 2.155e-12) and, via the Joseph K R K^T squaring loop, diverging
    * P. Any IMU whose gyro-noise trace is below SIGMA_GYRO_FLOOR_TRACE is floored to SIGMA_GYRO_FLOOR * I3.
    * <p>
    * 2026-07-10: lowered 1e-4 -&gt; 1e-6 (sigma 0.01 -&gt; 0.001 rad/s). This is a SAFETY NET only — the real gyro
    * Sigma is now wired via AlexSensorNoiseParameters and sits above it, so the floor should not normally engage.
    * The old 1e-4 was ~2500x the wired value; because the JointKF uses Sigma to weight the gyro in BOTH the
    * joint-velocity and the base-bias estimate, that over-inflation lazily degraded the base gyro-bias estimate
    * the downstream InEKF relies on. 1e-6 keeps lambda_min(S) ~6 orders above the 2e-12 collapse (the COND_S_MAX
    * gate backstops), while no longer over-inflating the gyro when a real (small) Sigma is present.
    */
   static final double SIGMA_GYRO_FLOOR = 1.0e-6;        // (0.001 rad/s)^2 per axis (safety net)
   /** 3 * (0.001 rad/s)^2 — the trace below which an IMU's gyro covariance is substituted by the floor. */
   static final double SIGMA_GYRO_FLOOR_TRACE = 3.0e-6;
   /**
    * Active innovation-covariance conditioning gate (Part B item 5). cond(S) is estimated from the Cholesky
    * factor diagonal ((max L_ii / min L_ii)^2 — no eigendecomposition); above this the whole stacked update is
    * skipped, because a finite-but-ill-conditioned S inverts to a huge gain that the Joseph K R K^T loop squares
    * each tick (the P divergence mechanism the LU/isFinite guards were blind to).
    */
   static final double COND_S_MAX = 1.0e9;
   /** Encoders trusted at initialization. */
   static final double INIT_POS_VAR = 1.0e-6;
   /** Velocity unknown at initialization. */
   static final double INIT_VEL_VAR = 1.0;
   /** (0.05 rad/s)^2. */
   static final double INIT_BIAS_VAR = 2.5e-3;
   /** Stance FK slip variance (Sigma_eps). */
   static final double ANCHOR_VAR = 4.0e-4;
   /**
    * Encoder joint-VELOCITY noise STD (rad/s) for leg joints on a base-&gt;foot chain that are NOT filter states
    * (on Alex: the ankles, because there are no foot IMUs). Their measured velocity enters the stance-anchor row
    * as a known input, so by the standard input-noise congruence its covariance must be propagated into the
    * anchor's measurement covariance:  R_anchor = Sigma_eps + J_U diag(SIGMA_QD_UNFILTERED^2) J_U^T.
    * <p>
    * Conservative placeholder: MEASURE the actual qd noise of the ankle encoder signal and retune. Erring large
    * is safe -- it merely weakens the anchor, which still fixes the bias gauge, just with more averaging. Erring
    * small is NOT safe: it over-trusts a noisy input and feeds that noise into the base gyro-bias estimate.
    */
   static final double SIGMA_QD_UNFILTERED = 0.1; // rad/s
   /**
    * Smoothing corner (Hz) for the measured-q̇ slew estimate that drives the direct-velocity channel's lag
    * inflation. The firmware/alpha low-pass makes q̇^meas a LAGGED measurement; for a first-order filter the
    * identity u - y = ẏ/ω_c is EXACT, so the instantaneous lag error is the measured signal's own slope over
    * the effective corner (cascade: 1/ω_eff = Σ 1/ω_stage). The slope must be estimated from a finite
    * difference of the NOISY measurement — raw, its variance 2σ²/dt² would inflate R by ~2 orders of magnitude
    * at quiet standing, exactly the regime the channel exists for — so it is low-passed here. 5 Hz sits above
    * the gait band (slew tracking stays honest through swing) while cutting the FD noise contribution to ~σ²
    * order. See the direct-velocity channel's refreshDirectVelocityNoise().
    */
   static final double LAG_SLEW_SMOOTHING_HZ = 5.0;
   /**
    * On-ground initialization gate: the exported base-IMU gyro bias is only observable through the stance
    * anchor, which runs only when a foot is trusted. If the filter is seeded while the robot hangs (feet off the
    * ground) the base bias is unobservable, its covariance grows unbounded under the bias random-walk, and the
    * estimate wanders — which the downstream InEKF (no gyro-bias state of its own) integrates straight into
    * orientation. So we defer initialization until BOTH feet have been firmly in contact for a short debounce
    * window; while uninitialized the filter exports NaN joint states and zero bias, so consumers cleanly fall
    * back to the raw gyro/sensors. Debounced (not a single-tick check) because the foot-switch
    * contact-probability source seeds to 1.0 on the assumption feet are planted at init, so a naive
    * "both == 1" would false-pass on the first tick(s) precisely while hanging.
    */
   static final double ON_GROUND_INIT_DEBOUNCE = 0.05; // s of continuous ground contact before seeding

   // ================================ Tier descriptions ================================

   private static final String LIVE = "LIVE: re-read every tick — edit mid-run to retune the filter. ";
   private static final String BOOT = "BOOT-TIME ONLY: consumed once at construction; editing mid-run has NO effect. ";
   private static final String INIT = "INIT-TIME: read when the filter seeds; editing has effect only before initialization. ";
   private static final String DOC = "REFERENCE ONLY: not consumed by the filter. ";

   // ================================ Live parameters ================================

   /** (rad/s^2)^2 tripwire on max diag(Qa). Surfaces a model/config regression; never rescales Qa. */
   final YoDouble qaMax;
   /** Innovation-covariance conditioning ceiling; above it the whole measurement block is skipped. */
   final YoDouble condSMax;
   /** (rad/s)^2 stance-anchor FK slip variance Sigma_eps. */
   final YoDouble anchorVar;
   /** (rad/s)^2 per-axis gyro measurement-noise floor, and the S-pivot floor for non-encoder blocks. */
   final YoDouble sigmaGyroFloor;

   // ================================ Boot-time parameters ================================

   final YoDouble encoderVar;
   final YoDouble sigmaAccel;
   final YoDouble sigmaTau;
   final YoDouble alphaDefault;
   final YoDouble rotorInertiaDefault;
   final YoDouble sigmaGyroFloorTrace;
   final YoDouble sigmaQdUnfiltered;
   final YoDouble lagSlewSmoothingHz;
   final YoDouble onGroundInitDebounce;
   final YoDouble initPosVar;
   final YoDouble initVelVar;
   final YoDouble initBiasVar;
   final YoDouble targetQddStd;

   JointKFParameters(YoRegistry registry)
   {
      qaMax = create("qaMax",
                     LIVE + "Physical ceiling on per-joint acceleration process-noise variance ((rad/s^2)^2). "
                     + "Exceeding it counts in jointKFQaCapWouldBindCount and warns; Qa is NOT rescaled.",
                     QA_MAX, registry);
      condSMax = create("condSMax",
                        LIVE + "Max cond(S) proxy (max L_ii / min L_ii)^2 before the whole measurement block is "
                        + "gated. Raising it re-admits ill-conditioned updates that the Joseph K R K^T loop squares each tick.",
                        COND_S_MAX, registry);
      anchorVar = create("anchorVar",
                         LIVE + "Stance-anchor FK slip variance Sigma_eps ((rad/s)^2), added to each active "
                         + "anchor's 3x3 block of R_g.",
                         ANCHOR_VAR, registry);
      sigmaGyroFloor = create("sigmaGyroFloor",
                              LIVE + "Gyro measurement-noise floor ((rad/s)^2 per axis). Also the S-pivot floor "
                              + "for every non-encoder block. A safety net: the wired Sigma should sit above it.",
                              SIGMA_GYRO_FLOOR, registry);

      encoderVar = create("encoderVar",
                          BOOT + "Fallback encoder position variance (rad^2) for joints with no wired per-joint "
                          + "noise. Orders of magnitude above measured values — check jointKF_encR_<joint>.",
                          ENCODER_VAR, registry);
      sigmaAccel = create("sigmaAccel",
                          BOOT + "Scalar-CWNA process-noise STD (rad/s^2), used only when no robot model is "
                          + "provided (no mass-matrix path).",
                          SIGMA_ACCEL, registry);
      sigmaTau = create("sigmaTau",
                        BOOT + "Fallback unmodeled-torque STD (N*m) for a joint with no finite positive effort "
                        + "limit. The live per-joint value is jointKFParam_alpha_<joint> * tau_max.",
                        SIGMA_TAU, registry);
      alphaDefault = create("alphaDefault",
                            BOOT + "Fallback alpha (fraction of tau_max that is unmodeled) for a filtered joint "
                            + "absent from the calibrated table. Seeds jointKFParam_alpha_<joint>.",
                            ALPHA_DEFAULT, registry);
      rotorInertiaDefault = create("rotorInertiaDefault",
                                   BOOT + "Conservative reflected-rotor-inertia floor (kg*m^2) for a joint absent "
                                   + "from the table. Seeds jointKFParam_rotorInertia_<joint>.",
                                   ROTOR_INERTIA_DEFAULT, registry);
      sigmaGyroFloorTrace = create("sigmaGyroFloorTrace",
                                   BOOT + "Gyro-noise trace ((rad/s)^2) below which an IMU's covariance is "
                                   + "substituted by sigmaGyroFloor * I3. Evaluated once, on the first tick.",
                                   SIGMA_GYRO_FLOOR_TRACE, registry);
      sigmaQdUnfiltered = create("sigmaQdUnfiltered",
                                 BOOT + "Fallback encoder velocity STD (rad/s) for joints with no wired value — "
                                 + "both the unfiltered anchor-chain joints and the direct-velocity channel floor.",
                                 SIGMA_QD_UNFILTERED, registry);
      lagSlewSmoothingHz = create("lagSlewSmoothingHz",
                                  BOOT + "Corner (Hz) of the low-pass on the measured-q̇ slew estimate driving "
                                  + "the direct-velocity lag inflation.",
                                  LAG_SLEW_SMOOTHING_HZ, registry);
      onGroundInitDebounce = create("onGroundInitDebounce",
                                    BOOT + "Seconds of continuous ground contact required before the filter "
                                    + "seeds. Converted to a tick count at construction.",
                                    ON_GROUND_INIT_DEBOUNCE, registry);
      initPosVar = create("initPosVar",
                          INIT + "Initial joint-position variance (rad^2) — encoders trusted at seeding.",
                          INIT_POS_VAR, registry);
      initVelVar = create("initVelVar",
                          INIT + "Initial joint-velocity variance ((rad/s)^2) — velocity unknown at seeding.",
                          INIT_VEL_VAR, registry);
      initBiasVar = create("initBiasVar",
                           INIT + "Initial per-IMU gyro-bias variance ((rad/s)^2) at seeding.",
                           INIT_BIAS_VAR, registry);
      targetQddStd = create("targetQddStd",
                            DOC + "Common unmodeled-acceleration STD target (rad/s^2) that the calibrated "
                            + "per-joint alphas were equalized to. Published so a log records the calibration target.",
                            TARGET_QDD_STD, registry);
   }

   private static YoDouble create(String name, String description, double defaultValue, YoRegistry registry)
   {
      YoDouble variable = new YoDouble("jointKFParam_" + name, description, registry);
      variable.set(defaultValue);
      return variable;
   }

   // ================================ Default table lookups ================================

   /** Reflected rotor inertia for a joint by case-insensitive name substring (Part B item 1); -1 if unmatched. */
   private static double lookupRotorInertia(String jointName)
   {
      String upper = jointName.toUpperCase();
      for (int i = 0; i < ROTOR_INERTIA_JOINT_KEYS.length; i++)
         if (upper.contains(ROTOR_INERTIA_JOINT_KEYS[i]))
            return ROTOR_INERTIA_VALUES[i];
      return -1.0;
   }

   /**
    * The reflected rotor inertia the filter applies to the named joint — the table match, or the conservative
    * default when unmatched. Lets the mass-matrix Qa oracle reconstruct Lambda_eff exactly (Part C), independent
    * of whether a synthetic joint name happens to hit a table key.
    */
   static double reflectedRotorInertiaForNameOrDefault(String jointName)
   {
      double v = lookupRotorInertia(jointName);
      return v < 0.0 ? ROTOR_INERTIA_DEFAULT : v;
   }

   /** True when the rotor-inertia table has no entry for this joint, i.e. the default floor is being applied. */
   static boolean isRotorInertiaUnmatched(String jointName)
   {
      return lookupRotorInertia(jointName) < 0.0;
   }

   /**
    * Per-joint alpha (fraction of tau_max used as the unmodeled-torque STD) by case-insensitive name substring;
    * the calibrated table value if matched, else ALPHA_DEFAULT.
    */
   static double alphaForName(String jointName)
   {
      String upper = jointName.toUpperCase();
      for (int i = 0; i < ALPHA_JOINT_KEYS.length; i++)
         if (upper.contains(ALPHA_JOINT_KEYS[i]))
            return ALPHA_VALUES[i];
      return ALPHA_DEFAULT;
   }
}

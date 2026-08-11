package us.ihmc.stateEstimation.jointLevel;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Every tuning constant of the joint-level KF, published as YoVariables so the calibration procedures below can
 * be run live in SCS instead of through a recompile.
 *
 * <p>ONE INSTANCE PER FILTER, never static: a YoVariable needs a registry, so a static holder would collide on
 * the second filter built in one JVM. The default tables stay static — they only seed the YoVariables.</p>
 *
 * <p>Each variable's description states its tier. LIVE ones ({@link #qaMax}, {@link #condSMax},
 * {@link #anchorVar}, {@link #sigmaGyroFloor}, and the per-joint alpha / rotor inertia) are re-read every tick;
 * the rest are consumed once at construction and published only for visibility. There are deliberately no
 * change listeners — one firing on the SCS GUI thread would mutate Q / R underneath the estimator thread.</p>
 *
 * @author Lucas Libshutz
 */
final class JointKFParameters
{
   // ================================ Defaults ================================

   /**
    * Fallback encoder position variance (rad^2) for a joint the per-joint lookup does not cover. Hardware
    * per-joint values run sigma 5.6e-5..7.5e-4 rad — 2-4 ORDERS OF MAGNITUDE below this — so a joint silently
    * on the fallback badly under-trusts its encoder. Watch jointKF_encR_&lt;joint&gt; at boot.
    */
   static final double ENCODER_VAR = 5.0e-5;
   /** rad/s^2 CWNA process-noise STD; the scalar fallback used only when no robot model is provided. */
   static final double SIGMA_ACCEL = 50.0;
   /**
    * FALLBACK unmodeled-torque STD (N*m), used only for a joint whose effort limit is absent/non-finite; the
    * live per-joint STD is sigma_tau,i = alpha_i * tau_max,i. Rev.2 interim value. TODO(retune) with ALPHA.
    */
   static final double SIGMA_TAU = 5.0;

   // Per-joint unmodeled-torque STD as a fraction of the joint's effort limit: sigma_tau,i = alpha_i * tau_max,i.
   // tau_max,i already scales by actuator capacity; alpha_i is the dimensionless "fraction of capacity that is
   // unmodeled", matched by case-insensitive name substring with a scalar default.
   //
   // RETUNE PRINCIPLE (2026-07-10, log 20260710_135507): a uniform alpha fixes unmodeled torque at a fixed
   // fraction of CAPACITY, but the torque->acceleration map Lambda_eff^-1 varies by orders of magnitude across
   // joints, so joints trip QA_MAX one after another. The fix is to equalize the unmodeled-ACCELERATION STD at a
   // common TARGET_QDD_STD:
   //     alpha_i = TARGET_QDD_STD / (|Lambda_eff^-1|_ii * tau_max,i) = alpha_old_i * sqrt(TARGET_QDD_STD^2 / diag(Qa)_i)
   // The second form is how to CALIBRATE from a run: read jointKF_QaDiag_<joint> at the current alpha and
   // rescale, iterating 2-3x (off-diagonal coupling makes it not one-shot). With alpha now LIVE as
   // jointKFParam_alpha_<joint>, that loop runs in SCS without a rebuild.
   //
   // ALPHA_VALUES below are the CALIBRATED equalized set (2026-07-10, live STAND_PREP read). Cross-check: the
   // LEFT/RIGHT pairs agree to ~0.2%, and the legs are physically identical, which validates the measurement.
   // diag(Qa) is CONFIGURATION-DEPENDENT, so re-read after a gait change and iterate until every
   // sqrt(diag(Qa)) ~ 20 and the jointKF_QaCapBind_<joint>_count counters stay flat.

   /** rad/s^2, the common unmodeled-acceleration STD the calibrated alphas were equalized to. */
   static final double TARGET_QDD_STD = 20.0;
   /** Fallback only; surfaces an unlisted filtered joint via the QA_MAX tripwire. */
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
    * TRIPWIRE ONLY, not a scaler: a physical ceiling (~(30 rad/s^2)^2) on the per-joint acceleration
    * process-noise VARIANCE. Exceeding it warns and counts but does NOT rescale Qa — the old uniform rescale
    * coupled one joint's outlier into global Q starvation, which CAUSED the min-side S singularity.
    */
   static final double QA_MAX = 900.0;

   // Reflected rotor inertia n^2 * J_rotor (kg*m^2) per joint, matched by case-insensitive name substring, added
   // to the Schur complement's diagonal BEFORE inversion: Lambda_eff = Lambda + diag(n_i^2 J_rotor,i). The rotor
   // spins behind the gearbox about its own axis and does not couple through the floating base, so this is
   // simultaneously the EXACT drivetrain term and a principled regularizer (it floors lambda_min by Weyl).
   // Without it Lambda^-2 has diagonal outliers up to ~1.6e6 and Qa blows up for proximal/light joints.
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
    * Gyro measurement-noise floor ((0.001 rad/s)^2 per axis). A zero Sigma — which is what an unset
    * SensorNoiseParameters yields — removes the innovation-covariance floor on the pure-bias rows of every 1-DoF
    * chain, collapsing lambda_min(S) and diverging P through the Joseph K R K^T loop. SAFETY NET only: the real
    * gyro Sigma is wired via AlexSensorNoiseParameters and sits above this, so it should not normally engage.
    */
   static final double SIGMA_GYRO_FLOOR = 1.0e-6;
   /** 3 * (0.001 rad/s)^2 — the trace below which an IMU's gyro covariance is substituted by the floor. */
   static final double SIGMA_GYRO_FLOOR_TRACE = 3.0e-6;
   /**
    * Innovation-covariance conditioning gate. cond(S) is estimated from the Cholesky factor diagonal
    * ((max L_ii / min L_ii)^2, no eigendecomposition); above this the whole update is skipped, because a
    * finite-but-ill-conditioned S inverts to a huge gain that the Joseph K R K^T loop squares each tick.
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
    * Encoder joint-VELOCITY noise STD (rad/s) for base-&gt;foot chain joints that are NOT filter states (on Alex
    * the ankles, since there are no foot IMUs). Their measured velocity enters the stance-anchor row as a known
    * input, so its covariance propagates in by congruence: R = Sigma_eps + J_U diag(sigma^2) J_U^T. Conservative
    * placeholder — erring large only weakens the anchor; erring small feeds encoder noise into the base bias.
    */
   static final double SIGMA_QD_UNFILTERED = 0.1;
   /**
    * Smoothing corner (Hz) for the measured-q̇ slew estimate driving the direct-velocity lag inflation. The slope
    * must be estimated from a finite difference of a NOISY measurement, whose raw variance 2*sigma^2/dt^2 would
    * inflate R by ~2 orders of magnitude at quiet standing; 5 Hz sits above the gait band while cutting that
    * contribution to ~sigma^2 order.
    */
   static final double LAG_SLEW_SMOOTHING_HZ = 5.0;
   /**
    * Seconds of continuous ground contact required before seeding. The exported base gyro bias is observable
    * only through the stance anchor, so seeding while the robot hangs lets it random-walk into the InEKF's
    * orientation. Debounced because the contact-probability source seeds to 1.0, so a single-tick check
    * false-passes on the first tick(s) precisely while hanging.
    */
   static final double ON_GROUND_INIT_DEBOUNCE = 0.05;

   private static final String LIVE = "LIVE: re-read every tick — edit mid-run to retune the filter. ";
   private static final String BOOT = "BOOT-TIME ONLY: consumed once at construction; editing mid-run has NO effect. ";
   private static final String INIT = "INIT-TIME: read when the filter seeds; editing has effect only before initialization. ";
   private static final String DOC = "REFERENCE ONLY: not consumed by the filter. ";

   // ================================ Live ================================

   final YoDouble qaMax;
   final YoDouble condSMax;
   final YoDouble anchorVar;
   final YoDouble sigmaGyroFloor;

   // ================================ Boot-time ================================

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

   /** Reflected rotor inertia by case-insensitive name substring; -1 if unmatched. */
   private static double lookupRotorInertia(String jointName)
   {
      String upper = jointName.toUpperCase();
      for (int i = 0; i < ROTOR_INERTIA_JOINT_KEYS.length; i++)
         if (upper.contains(ROTOR_INERTIA_JOINT_KEYS[i]))
            return ROTOR_INERTIA_VALUES[i];
      return -1.0;
   }

   /**
    * The rotor inertia the filter actually applies: the table match, or the conservative default when
    * unmatched. Lets the mass-matrix Qa oracle reconstruct Lambda_eff exactly for synthetic joint names.
    */
   static double reflectedRotorInertiaForNameOrDefault(String jointName)
   {
      double v = lookupRotorInertia(jointName);
      return v < 0.0 ? ROTOR_INERTIA_DEFAULT : v;
   }

   /** True when the table has no entry for this joint, i.e. the default floor is being applied. */
   static boolean isRotorInertiaUnmatched(String jointName)
   {
      return lookupRotorInertia(jointName) < 0.0;
   }

   /** Per-joint alpha by case-insensitive name substring; the calibrated table value, else ALPHA_DEFAULT. */
   static double alphaForName(String jointName)
   {
      String upper = jointName.toUpperCase();
      for (int i = 0; i < ALPHA_JOINT_KEYS.length; i++)
         if (upper.contains(ALPHA_JOINT_KEYS[i]))
            return ALPHA_VALUES[i];
      return ALPHA_DEFAULT;
   }
}

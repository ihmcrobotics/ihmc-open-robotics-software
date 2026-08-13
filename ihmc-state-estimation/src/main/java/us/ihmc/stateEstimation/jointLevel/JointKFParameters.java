package us.ihmc.stateEstimation.jointLevel;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Every tuning constant of the joint-level KF, published as YoVariables so it can be retuned live in SCS.
 * ONE INSTANCE PER FILTER, never static: a YoVariable needs a registry, so a static holder would collide on
 * the second filter built in one JVM. Each variable's description states its tier — LIVE is re-read every
 * tick, the rest are consumed once. Deliberately no change listeners: one firing on the SCS GUI thread would
 * mutate Q / R underneath the estimator thread. Rationale for the values: CHANGES.md, "Parameter tuning
 * reference".
 *
 * @author Lucas Libshutz
 */
final class JointKFParameters
{
   // ================================ Defaults ================================

   /** Fallback encoder position variance (rad^2). Sits 2-4 orders above measured — watch jointKF_encR_&lt;joint&gt;. */
   static final double ENCODER_VAR = 5.0e-5;
   /** Scalar CWNA process-noise STD (rad/s^2); the fallback used only when no robot model is provided. */
   static final double SIGMA_ACCEL = 50.0;
   /** Fallback unmodeled-torque STD (N*m) for a joint with no finite effort limit. TODO(retune) with ALPHA. */
   static final double SIGMA_TAU = 5.0;

   /** rad/s^2, the common unmodeled-acceleration STD the calibrated alphas were equalized to. */
   static final double TARGET_QDD_STD = 20.0;
   /** Fallback alpha; surfaces an unlisted filtered joint via the QA_MAX tripwire. */
   static final double ALPHA_DEFAULT = 0.15;

   // Per-joint unmodeled-torque fraction: sigma_tau,i = alpha_i * tau_max,i. Calibrated equalized set
   // (2026-07-10); re-read diag(Qa) after a gait change and iterate. Procedure in CHANGES.md.
   private static final String[] ALPHA_JOINT_KEYS = {
         "SPINE_Z",
         "LEFT_HIP_X",  "LEFT_HIP_Z",  "LEFT_HIP_Y",  "LEFT_KNEE_Y",
         "RIGHT_HIP_X", "RIGHT_HIP_Z", "RIGHT_HIP_Y", "RIGHT_KNEE_Y"};
   private static final double[] ALPHA_VALUES = {
         5.61133e-2,
         6.01544e-2, 2.94985e-2, 2.82043e-2, 2.47925e-2,
         5.52493e-2, 3.34146e-2, 2.56966e-2, 2.49771e-2
   };

   /** Per-joint acceleration process-noise variance ceiling ((rad/s^2)^2). Tripwire only — never rescales Qa. */
   static final double QA_MAX = 900.0;

   // Reflected rotor inertia n^2 * J_rotor (kg*m^2), added to the Schur complement's diagonal before
   // inversion: Lambda_eff = Lambda + diag(n_i^2 J_rotor,i). Exact drivetrain term and Weyl floor; CHANGES.md.
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

   /** Gyro measurement-noise floor ((0.001 rad/s)^2 per axis). Safety net against a zero Sigma; CHANGES.md. */
   static final double SIGMA_GYRO_FLOOR = 1.0e-6;
   /** 3 * (0.001 rad/s)^2 — trace below which an IMU's gyro covariance is substituted by the floor. */
   static final double SIGMA_GYRO_FLOOR_TRACE = 3.0e-6;
   /** Innovation-covariance conditioning gate on the Cholesky-diagonal cond(S) proxy; CHANGES.md. */
   static final double COND_S_MAX = 1.0e9;
   /** Initial joint-position variance (rad^2) — encoders trusted at initialization. */
   static final double INIT_POS_VAR = 1.0e-6;
   /** Initial joint-velocity variance ((rad/s)^2) — velocity unknown at initialization. */
   static final double INIT_VEL_VAR = 1.0;
   /** Initial per-IMU gyro-bias variance ((rad/s)^2), i.e. (0.05 rad/s)^2. */
   static final double INIT_BIAS_VAR = 2.5e-3;
   /** Stance FK slip variance Sigma_eps ((rad/s)^2). */
   static final double ANCHOR_VAR = 4.0e-4;
   /** Encoder velocity STD (rad/s) for unfiltered base-&gt;foot chain joints; CHANGES.md. */
   static final double SIGMA_QD_UNFILTERED = 0.1;
   /** Smoothing corner (Hz) for the measured-q̇ slew driving the direct-velocity lag inflation; CHANGES.md. */
   static final double LAG_SLEW_SMOOTHING_HZ = 5.0;
   /** Seconds of continuous ground contact required before seeding; CHANGES.md. */
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
      qaMax = create("qaMax", LIVE + "Per-joint acceleration process-noise variance ceiling ((rad/s^2)^2). "
                              + "Counts in jointKFQaCapWouldBindCount and warns; Qa is NOT rescaled.", QA_MAX, registry);
      condSMax = create("condSMax", LIVE + "Max cond(S) proxy (max L_ii / min L_ii)^2 before the measurement "
                                    + "block is gated.", COND_S_MAX, registry);
      anchorVar = create("anchorVar", LIVE + "Stance-anchor FK slip variance Sigma_eps ((rad/s)^2).", ANCHOR_VAR, registry);
      sigmaGyroFloor = create("sigmaGyroFloor", LIVE + "Gyro measurement-noise floor ((rad/s)^2 per axis), and "
                                                + "the S-pivot floor for every non-encoder block.", SIGMA_GYRO_FLOOR, registry);

      encoderVar = create("encoderVar", BOOT + "Fallback encoder position variance (rad^2) for joints with no "
                                        + "wired per-joint noise.", ENCODER_VAR, registry);
      sigmaAccel = create("sigmaAccel", BOOT + "Scalar-CWNA process-noise STD (rad/s^2), used only when no robot "
                                        + "model is provided.", SIGMA_ACCEL, registry);
      sigmaTau = create("sigmaTau", BOOT + "Fallback unmodeled-torque STD (N*m) for a joint with no finite "
                                    + "positive effort limit.", SIGMA_TAU, registry);
      alphaDefault = create("alphaDefault", BOOT + "Fallback alpha (fraction of tau_max that is unmodeled). "
                                            + "Seeds jointKFParam_alpha_<joint>.", ALPHA_DEFAULT, registry);
      rotorInertiaDefault = create("rotorInertiaDefault", BOOT + "Conservative reflected-rotor-inertia floor "
                                                          + "(kg*m^2). Seeds jointKFParam_rotorInertia_<joint>.", ROTOR_INERTIA_DEFAULT, registry);
      sigmaGyroFloorTrace = create("sigmaGyroFloorTrace", BOOT + "Gyro-noise trace ((rad/s)^2) below which an "
                                                          + "IMU's covariance is substituted by sigmaGyroFloor * I3.", SIGMA_GYRO_FLOOR_TRACE, registry);
      sigmaQdUnfiltered = create("sigmaQdUnfiltered", BOOT + "Fallback encoder velocity STD (rad/s) for joints "
                                                      + "with no wired value.", SIGMA_QD_UNFILTERED, registry);
      lagSlewSmoothingHz = create("lagSlewSmoothingHz", BOOT + "Corner (Hz) of the low-pass on the measured-q̇ "
                                                        + "slew estimate.", LAG_SLEW_SMOOTHING_HZ, registry);
      onGroundInitDebounce = create("onGroundInitDebounce", BOOT + "Seconds of continuous ground contact required "
                                                            + "before the filter seeds.", ON_GROUND_INIT_DEBOUNCE, registry);
      initPosVar = create("initPosVar", INIT + "Initial joint-position variance (rad^2).", INIT_POS_VAR, registry);
      initVelVar = create("initVelVar", INIT + "Initial joint-velocity variance ((rad/s)^2).", INIT_VEL_VAR, registry);
      initBiasVar = create("initBiasVar", INIT + "Initial per-IMU gyro-bias variance ((rad/s)^2).", INIT_BIAS_VAR, registry);
      targetQddStd = create("targetQddStd", DOC + "Common unmodeled-acceleration STD target (rad/s^2) the "
                                             + "calibrated per-joint alphas were equalized to.", TARGET_QDD_STD, registry);
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

   /** The rotor inertia the filter applies: the table match, or the conservative default when unmatched. */
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

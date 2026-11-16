package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

/**
 * Which pelvis pose corrections in the state estimator's {@link PelvisPoseCorrectionSeries} are active. The series always holds
 * the native corrector first and the EKF pelvis pose correction second; this covers every combination of the two.
 * <p>
 * It sets the starting values of each correction's active yo-variable, which can still be changed at runtime.
 * </p>
 */
public enum PelvisPoseCorrectionMode
{
   /** No correction: the estimator's pelvis pose is plain kinematic-inertial odometry. */
   NONE(false, false),
   /** The native pelvis pose corrector, {@link NewPelvisPoseHistoryCorrection}. */
   NATIVE(true, false),
   /** The EKF pelvis pose correction, fusing the estimator's motion with the external poses. */
   EKF(false, true),
   /** The native corrector, then the EKF refining the pose the native corrector left. */
   NATIVE_THEN_EKF(true, true);

   private final boolean usesNativeCorrection;
   private final boolean usesEKFCorrection;

   PelvisPoseCorrectionMode(boolean usesNativeCorrection, boolean usesEKFCorrection)
   {
      this.usesNativeCorrection = usesNativeCorrection;
      this.usesEKFCorrection = usesEKFCorrection;
   }

   public boolean usesNativeCorrection()
   {
      return usesNativeCorrection;
   }

   public boolean usesEKFCorrection()
   {
      return usesEKFCorrection;
   }

   /** Whether any correction is active, so the estimator uses external poses. */
   public boolean usesExternalPoses()
   {
      return usesNativeCorrection || usesEKFCorrection;
   }

   /** Switches the two corrections of the series on or off to match this mode. */
   public void apply(PelvisPoseHistoryCorrectionInterface nativeCorrection, PelvisPoseHistoryCorrectionInterface ekfCorrection)
   {
      nativeCorrection.setActive(usesNativeCorrection);
      ekfCorrection.setActive(usesEKFCorrection);
   }
}

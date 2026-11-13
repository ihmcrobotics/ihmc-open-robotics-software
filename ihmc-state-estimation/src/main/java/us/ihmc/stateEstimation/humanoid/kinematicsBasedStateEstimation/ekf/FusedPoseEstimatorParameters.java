package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

/** Tuning for {@link FusedPoseEstimator}. Read once, when the estimator is created. */
public class FusedPoseEstimatorParameters
{
   private double positionProcessStandardDeviation = 0.10;
   private double angleProcessStandardDeviation = Math.toRadians(5.0);
   private double positionMeasurementStandardDeviation = 0.10;
   private double angleMeasurementStandardDeviation = Math.toRadians(5.0);
   private double translationGate = 0.20;
   private double rotationGate = Math.toRadians(15.0);
   private double maximumCorrectionTranslationSpeed = 0.5;
   private double maximumCorrectionRotationSpeed = 0.5;
   private long maximumMeasurementTimeOffset = 200_000_000L;
   private long maximumClockDisagreement = 5_000_000L;

   /** How fast the kinematic position estimate may drift, in m per sqrt(s). */
   public double getPositionProcessStandardDeviation()
   {
      return positionProcessStandardDeviation;
   }

   public void setPositionProcessStandardDeviation(double positionProcessStandardDeviation)
   {
      this.positionProcessStandardDeviation = positionProcessStandardDeviation;
   }

   /** How fast the kinematic orientation estimate may drift, in rad per sqrt(s). */
   public double getAngleProcessStandardDeviation()
   {
      return angleProcessStandardDeviation;
   }

   public void setAngleProcessStandardDeviation(double angleProcessStandardDeviation)
   {
      this.angleProcessStandardDeviation = angleProcessStandardDeviation;
   }

   /** The position error of one pose measurement, in m. */
   public double getPositionMeasurementStandardDeviation()
   {
      return positionMeasurementStandardDeviation;
   }

   public void setPositionMeasurementStandardDeviation(double positionMeasurementStandardDeviation)
   {
      this.positionMeasurementStandardDeviation = positionMeasurementStandardDeviation;
   }

   /** The orientation error of one pose measurement, in rad. */
   public double getAngleMeasurementStandardDeviation()
   {
      return angleMeasurementStandardDeviation;
   }

   public void setAngleMeasurementStandardDeviation(double angleMeasurementStandardDeviation)
   {
      this.angleMeasurementStandardDeviation = angleMeasurementStandardDeviation;
   }

   /** A pose measurement further than this from the prediction, in m, is rejected. */
   public double getTranslationGate()
   {
      return translationGate;
   }

   public void setTranslationGate(double translationGate)
   {
      this.translationGate = translationGate;
   }

   /** A pose measurement rotated further than this from the prediction, in rad, is rejected. */
   public double getRotationGate()
   {
      return rotationGate;
   }

   public void setRotationGate(double rotationGate)
   {
      this.rotationGate = rotationGate;
   }

   /** The fastest the output may move toward the corrected position, in m/s. */
   public double getMaximumCorrectionTranslationSpeed()
   {
      return maximumCorrectionTranslationSpeed;
   }

   public void setMaximumCorrectionTranslationSpeed(double maximumCorrectionTranslationSpeed)
   {
      this.maximumCorrectionTranslationSpeed = maximumCorrectionTranslationSpeed;
   }

   /** The fastest the output may rotate toward the corrected orientation, in rad/s. */
   public double getMaximumCorrectionRotationSpeed()
   {
      return maximumCorrectionRotationSpeed;
   }

   public void setMaximumCorrectionRotationSpeed(double maximumCorrectionRotationSpeed)
   {
      this.maximumCorrectionRotationSpeed = maximumCorrectionRotationSpeed;
   }

   /** A pose measurement whose exposure time is further than this from the current wall time, in ns, is ignored. */
   public long getMaximumMeasurementTimeOffset()
   {
      return maximumMeasurementTimeOffset;
   }

   public void setMaximumMeasurementTimeOffset(long maximumMeasurementTimeOffset)
   {
      this.maximumMeasurementTimeOffset = maximumMeasurementTimeOffset;
   }

   /**
    * If the wall and monotonic time steps of one tick differ by more than this, in ns, the clocks are considered faulty and
    * pose measurements are ignored until {@link FusedPoseEstimator#reset()}.
    */
   public long getMaximumClockDisagreement()
   {
      return maximumClockDisagreement;
   }

   public void setMaximumClockDisagreement(long maximumClockDisagreement)
   {
      this.maximumClockDisagreement = maximumClockDisagreement;
   }
}

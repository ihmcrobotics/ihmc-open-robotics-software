package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

/**
 * Initial tuning for {@link EKFPoseFuser}. Read once, when the fuser is created; the values become yo-variables, which
 * can be changed at runtime.
 */
public class EKFPoseFusionParameters
{
   private double positionProcessStandardDeviation = 0.10;
   private double orientationProcessStandardDeviation = Math.toRadians(5.0);
   private double positionMeasurementStandardDeviation = 0.10;
   private double orientationMeasurementStandardDeviation = Math.toRadians(5.0);
   private double maximumPositionInnovation = 0.20;
   private double maximumOrientationInnovation = Math.toRadians(15.0);
   /** The chi-square value for six degrees of freedom at 99.9 %. */
   private double maximumNormalizedInnovationSquared = 22.458;
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
   public double getOrientationProcessStandardDeviation()
   {
      return orientationProcessStandardDeviation;
   }

   public void setOrientationProcessStandardDeviation(double orientationProcessStandardDeviation)
   {
      this.orientationProcessStandardDeviation = orientationProcessStandardDeviation;
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
   public double getOrientationMeasurementStandardDeviation()
   {
      return orientationMeasurementStandardDeviation;
   }

   public void setOrientationMeasurementStandardDeviation(double orientationMeasurementStandardDeviation)
   {
      this.orientationMeasurementStandardDeviation = orientationMeasurementStandardDeviation;
   }

   /** A pose measurement further than this from the prediction, in m, is rejected. */
   public double getMaximumPositionInnovation()
   {
      return maximumPositionInnovation;
   }

   public void setMaximumPositionInnovation(double maximumPositionInnovation)
   {
      this.maximumPositionInnovation = maximumPositionInnovation;
   }

   /** A pose measurement rotated further than this from the prediction, in rad, is rejected. */
   public double getMaximumOrientationInnovation()
   {
      return maximumOrientationInnovation;
   }

   public void setMaximumOrientationInnovation(double maximumOrientationInnovation)
   {
      this.maximumOrientationInnovation = maximumOrientationInnovation;
   }

   /** A pose measurement whose normalized innovation squared is larger than this is rejected as statistically inconsistent. */
   public double getMaximumNormalizedInnovationSquared()
   {
      return maximumNormalizedInnovationSquared;
   }

   public void setMaximumNormalizedInnovationSquared(double maximumNormalizedInnovationSquared)
   {
      this.maximumNormalizedInnovationSquared = maximumNormalizedInnovationSquared;
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
    * pose measurements are ignored until {@link EKFPoseFuser#reset()}.
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

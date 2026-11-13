package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

import ihmc_common_msgs.StampedPosePacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

/**
 * Fuses a kinematic pose estimate with delayed pose measurements, for example from visual-inertial odometry, using a
 * {@link PoseErrorEKF}. A bounded history of kinematic pose estimates compensates each measurement's delay, each measurement
 * corrects the filter once, and an {@link EKFOutputSmoother} limits how fast the result moves.
 */
public final class FusedPoseEstimator
{
   private final long maximumMeasurementTimeOffset, maximumClockDisagreement;
   private final PoseHistory poseHistory = new PoseHistory();
   private final PoseErrorEKF filter;
   private final EKFOutputSmoother output;
   private final RigidBodyTransform kinematicPoseAtExposure = new RigidBodyTransform();
   private final RigidBodyTransform kinematicMotionSinceExposure = new RigidBodyTransform();
   private final RigidBodyTransform delayCompensatedMeasuredPose = new RigidBodyTransform();
   private final RigidBodyTransform posterior = new RigidBodyTransform();
   private final StringBuilder measurementFrameId = new StringBuilder();
   private boolean hasMeasurementFrame;
   private long previousWallTime, previousMonotonicTime;
   private volatile boolean clockFault;

   public FusedPoseEstimator()
   {
      this(new FusedPoseEstimatorParameters());
   }

   public FusedPoseEstimator(FusedPoseEstimatorParameters parameters)
   {
      maximumMeasurementTimeOffset = parameters.getMaximumMeasurementTimeOffset();
      maximumClockDisagreement = parameters.getMaximumClockDisagreement();
      filter = new PoseErrorEKF(parameters.getPositionProcessStandardDeviation(),
                                parameters.getAngleProcessStandardDeviation(),
                                parameters.getPositionMeasurementStandardDeviation(),
                                parameters.getAngleMeasurementStandardDeviation(),
                                parameters.getTranslationGate(),
                                parameters.getRotationGate());
      output = new EKFOutputSmoother(parameters.getMaximumCorrectionTranslationSpeed(), parameters.getMaximumCorrectionRotationSpeed());
      reset();
   }

   public void reset()
   {
      poseHistory.clear();
      filter.reset();
      output.reset();
      measurementFrameId.setLength(0);
      hasMeasurementFrame = false;
      previousWallTime = previousMonotonicTime = 0;
      clockFault = false;
   }

   /**
    * Both times are in nanoseconds. The wall time is matched against the pose measurement's exposure time, which may come from
    * another machine's clock; the monotonic time drives the filter and smoother, which need time steps that never jump. The
    * two are compared every tick; if they drift apart, pose measurements are ignored until {@link #reset()}.
    *
    * @param poseMeasurement  the new pose measurement, or {@code null} if there is none this tick. Its timestamp is the
    *                         wall time of its exposure, and its frame ID names the world it is expressed in: the first
    *                         measurement in a frame aligns that world to the current estimate. Not modified.
    * @param fusedPoseToPack the fused pose. Modified.
    */
   public void update(long wallTime,
                      long monotonicTime,
                      RigidBodyTransformReadOnly kinematicPoseEstimate,
                      StampedPosePacket poseMeasurement,
                      RigidBodyTransform fusedPoseToPack)
   {
      if (previousMonotonicTime != 0)
      {
         long wallTimeStep = wallTime - previousWallTime;
         long monotonicTimeStep = monotonicTime - previousMonotonicTime;
         // Wall time can be millisecond-quantized, so two valid high-rate ticks can share a wall time.
         if (monotonicTimeStep <= 0 || wallTimeStep < 0 || Math.abs(wallTimeStep - monotonicTimeStep) > maximumClockDisagreement)
            clockFault = true;
         if (monotonicTimeStep <= 0)
            throw new IllegalArgumentException("EKF requires increasing monotonic state time");
      }
      previousWallTime = wallTime;
      previousMonotonicTime = monotonicTime;
      poseHistory.add(wallTime, kinematicPoseEstimate);
      boolean hasMeasurement = false;
      if (!clockFault && poseMeasurement != null && isWithinMaximumTimeOffset(poseMeasurement.getTimestamp(), wallTime))
      {
         if (poseHistory.getPoseAtTime(poseMeasurement.getTimestamp(), kinematicPoseAtExposure))
         {
            if (!hasMeasurementFrame || CharSequence.compare(poseMeasurement.getFrameId(), measurementFrameId) != 0)
            {
               filter.resetMeasurementAlignment();
               measurementFrameId.setLength(0);
               measurementFrameId.append(poseMeasurement.getFrameId());
               hasMeasurementFrame = true;
            }
            // The measured pose is from the exposure time. Move it forward to now by the kinematic motion since then, not by a
            // corrected or fused output, so the measurement stays independent of the filter.
            kinematicMotionSinceExposure.setAndInvert(kinematicPoseAtExposure);
            kinematicMotionSinceExposure.multiply(kinematicPoseEstimate);
            poseMeasurement.getPose().getPose().get(delayCompensatedMeasuredPose);
            delayCompensatedMeasuredPose.multiply(kinematicMotionSinceExposure);
            hasMeasurement = true;
         }
      }
      filter.update(monotonicTime, kinematicPoseEstimate, hasMeasurement ? delayCompensatedMeasuredPose : null, posterior);
      output.update(monotonicTime, kinematicPoseEstimate, posterior, fusedPoseToPack);
   }

   private boolean isWithinMaximumTimeOffset(long timestamp, long currentWallTime)
   {
      return timestamp > 0 && timestamp >= currentWallTime - maximumMeasurementTimeOffset && timestamp <= currentWallTime + maximumMeasurementTimeOffset;
   }

   public String getStatus()
   {
      return (clockFault ? "CLOCK FAULT; pose measurements paused; " : "") + filter.getStatus();
   }
}

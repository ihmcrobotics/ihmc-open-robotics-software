package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import ihmc_common_msgs.StampedPosePacket;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointReadOnly;
import us.ihmc.robotics.kinematics.PoseHistory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * Rejects external pelvis poses that are too far from the state estimator's own pelvis pose at the same time, before any pelvis
 * pose correction sees them.
 * <p>
 * Each tick, after the pelvis pose corrections have run, {@link #recordEstimatedPelvisPose(long)} stores the estimator's output
 * pelvis pose, the same pose the robot state publishes. An external pose is compared with that history, interpolated at its
 * timestamp. Comparing with the output instead of the uncorrected KIO pose keeps the check from rejecting everything once KIO
 * has drifted further than the limits.
 * </p>
 * <p>
 * A pose is rejected when it contains NaN, when its timestamp isn't covered by the history, or when it's further than
 * {@code maximumExternalPelvisPositionError} or {@code maximumExternalPelvisOrientationError} from the estimated pose. The
 * limits are yo-variables, and the errors and counts are logged with them. Allocation-free after construction.
 * </p>
 */
public class StampedPosePacketValidator
{
   /** Estimator ticks of output pelvis pose kept, so an external pose's timestamp can be looked up despite its latency. */
   private static final int ESTIMATED_PELVIS_POSE_HISTORY_CAPACITY = 1000;
   private static final double DEFAULT_MAXIMUM_POSITION_ERROR = 0.20;
   private static final double DEFAULT_MAXIMUM_ORIENTATION_ERROR = Math.toRadians(15.0);

   private final FloatingJointReadOnly rootJoint;
   private final PoseHistory estimatedPelvisPoseHistory = new PoseHistory(ESTIMATED_PELVIS_POSE_HISTORY_CAPACITY);
   private final Pose3D estimatedPelvisPoseAtPacketTime = new Pose3D();

   private final YoDouble maximumPositionError;
   private final YoDouble maximumOrientationError;
   private final YoDouble lastPositionError;
   private final YoDouble lastOrientationError;
   private final YoLong validPoseCount;
   private final YoLong rejectedPoseCount;

   /**
    * @param rootJoint      the estimator's root joint, whose pose is the estimator's output pelvis pose.
    * @param parentRegistry the registry the limits, errors and counts are added to.
    */
   public StampedPosePacketValidator(FloatingJointReadOnly rootJoint, YoRegistry parentRegistry)
   {
      this.rootJoint = rootJoint;
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      maximumPositionError = new YoDouble("maximumExternalPelvisPositionError", registry);
      maximumOrientationError = new YoDouble("maximumExternalPelvisOrientationError", registry);
      lastPositionError = new YoDouble("lastExternalPelvisPositionError", registry);
      lastOrientationError = new YoDouble("lastExternalPelvisOrientationError", registry);
      validPoseCount = new YoLong("validExternalPelvisPoseCount", registry);
      rejectedPoseCount = new YoLong("rejectedExternalPelvisPoseCount", registry);
      maximumPositionError.set(DEFAULT_MAXIMUM_POSITION_ERROR);
      maximumOrientationError.set(DEFAULT_MAXIMUM_ORIENTATION_ERROR);
      parentRegistry.addChild(registry);
   }

   /** Records the root joint's current pose as the estimator's output at {@code timestamp}. Call once per tick, after the corrections. */
   public void recordEstimatedPelvisPose(long timestamp)
   {
      estimatedPelvisPoseHistory.add(timestamp, rootJoint.getJointPose());
   }

   /** Whether {@code stampedPosePacket} is close enough to the estimator's output pelvis pose at its timestamp to be used. */
   public boolean isValid(StampedPosePacket stampedPosePacket)
   {
      if (stampedPosePacket.getPose().getPose().containsNaN()
          || !estimatedPelvisPoseHistory.getPoseAtTime(stampedPosePacket.getTimestamp(), estimatedPelvisPoseAtPacketTime))
      {
         rejectedPoseCount.increment();
         return false;
      }

      lastPositionError.set(stampedPosePacket.getPose().getPose().getPosition().distance(estimatedPelvisPoseAtPacketTime.getPosition()));
      lastOrientationError.set(stampedPosePacket.getPose().getPose().getOrientation().distance(estimatedPelvisPoseAtPacketTime.getOrientation()));
      if (lastPositionError.getValue() > maximumPositionError.getValue() || lastOrientationError.getValue() > maximumOrientationError.getValue())
      {
         rejectedPoseCount.increment();
         return false;
      }

      validPoseCount.increment();
      return true;
   }
}

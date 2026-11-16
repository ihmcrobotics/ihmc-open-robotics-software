package us.ihmc.perception.vio;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotics.kinematics.PoseHistory;

import java.util.ArrayDeque;

/**
 * Turns VIO camera tracking poses into pelvis pose measurements in the robot's world, independent of ROS and camera hardware.
 * <p>
 * Robot states, the pelvis and head camera poses computed from the controller's robot state, go into a {@link PoseHistory} each.
 * Robot states arrive slower than camera poses, so each camera tracking pose waits until a robot state newer than its exposure
 * time arrives. It's then paired with the pelvis and head camera poses interpolated between the robot states just before and just
 * after its exposure time. The first camera pose after {@link #resetForNewSession()} sets the anchor: the transform from the VIO
 * world to the robot's world that puts that camera pose on the robot state's camera pose, so the error at the reset is zero and
 * later measurements show only KIO drift since then. Every later camera pose is moved by the same anchor and converted to a
 * pelvis pose.
 * </p>
 */
public class VIORootPoseCalculator
{
   /** Camera poses older than this, relative to the newest robot state, are rejected. */
   private static final long MAXIMUM_EXPOSURE_AGE_NANOSECONDS = 200_000_000L;
   /** When this many camera poses are waiting for a newer robot state, the oldest is dropped to make room. */
   private static final int PENDING_CAMERA_POSE_CAPACITY = 10;
   /**
    * Receives every converted pose. Whether a root pose is close enough to the estimator's own pose to be used is decided in the
    * controller process, by {@code StampedPosePacketValidator}.
    */
   @FunctionalInterface
   interface ConvertedPoseConsumer
   {
      void accept(long exposureTimestamp, RigidBodyTransformReadOnly rootPoseInControllerWorld, RigidBodyTransformReadOnly cameraPoseInControllerWorld);
   }

   /** A camera tracking pose waiting for a robot state newer than its exposure time. */
   private record IncomingCameraPose(long exposureTimestamp, RigidBodyTransform trackedCameraPoseInVIOWorld, RigidBodyTransform leftCameraOffset)
   {
      private IncomingCameraPose(long exposureTimestamp, RigidBodyTransformReadOnly trackedCameraPoseInVIOWorld, RigidBodyTransformReadOnly leftCameraOffset)
      {
         this(exposureTimestamp, new RigidBodyTransform(trackedCameraPoseInVIOWorld), new RigidBodyTransform(leftCameraOffset));
      }
   }

   private final PoseHistory controllerRootPoseHistory = new PoseHistory();
   private final PoseHistory controllerCameraPoseHistory = new PoseHistory();
   private long latestRobotStateTimestamp = Long.MIN_VALUE;
   /** Oldest first. */
   private final ArrayDeque<IncomingCameraPose> incomingVIOCameraPoses = new ArrayDeque<>(PENDING_CAMERA_POSE_CAPACITY);
   private final RigidBodyTransform vioWorldToRobotWorld = new RigidBodyTransform();
   private boolean areWorldFramesAnchoredToOneAnother;
   private final ConvertedPoseConsumer convertedPoseConsumer;
   private long acceptedCount;
   private long rejectedCount;
   private String statusReason = "waiting for paired input";

   VIORootPoseCalculator(ConvertedPoseConsumer convertedPoseConsumer)
   {
      this.convertedPoseConsumer = convertedPoseConsumer;
   }

   /** Clears the robot state history, pending camera poses and the anchor, so the next camera pose sets a new anchor. */
   synchronized void resetForNewSession()
   {
      controllerRootPoseHistory.clear();
      controllerCameraPoseHistory.clear();
      latestRobotStateTimestamp = Long.MIN_VALUE;
      incomingVIOCameraPoses.clear();
      areWorldFramesAnchoredToOneAnother = false;
      acceptedCount = 0;
      rejectedCount = 0;
      statusReason = "waiting for new input";
   }

   synchronized boolean areWorldFramesAnchoredToOneAnother()
   {
      return areWorldFramesAnchoredToOneAnother;
   }

   /**
    * A robot state: the root and head camera poses in the robot's world, from the controller, at {@code robotStateTimestamp}. A state that isn't newer
    * than the last one is ignored.
    */
   synchronized void addRobotControllerState(long robotStateTimestamp, RigidBodyTransformReadOnly rootPoseInWorld, RigidBodyTransformReadOnly cameraPoseInWorld)
   {
      if (robotStateTimestamp <= 0 || rootPoseInWorld == null || cameraPoseInWorld == null || rootPoseInWorld.containsNaN() || cameraPoseInWorld.containsNaN())
      {
         recordRejection("Invalid robot state from controller");
         return;
      }
      if (robotStateTimestamp <= latestRobotStateTimestamp)
         return;
      controllerRootPoseHistory.add(robotStateTimestamp, rootPoseInWorld);
      controllerCameraPoseHistory.add(robotStateTimestamp, cameraPoseInWorld);
      latestRobotStateTimestamp = robotStateTimestamp;
      processIncomingCameraPoses();
   }

   /** A camera tracking pose in the VIO world at {@code exposureTimestamp}, with the SDK's left-camera offset. */
   synchronized void addCameraTrackingPose(long exposureTimestamp,
                                           RigidBodyTransformReadOnly trackedCameraPoseInVIOWorld,
                                           RigidBodyTransformReadOnly leftCameraOffset)
   {
      if (trackedCameraPoseInVIOWorld == null || leftCameraOffset == null || trackedCameraPoseInVIOWorld.containsNaN() || leftCameraOffset.containsNaN())
      {
         recordRejection("invalid/lost VIO");
         return;
      }
      if (incomingVIOCameraPoses.size() >= PENDING_CAMERA_POSE_CAPACITY)
      {
         incomingVIOCameraPoses.removeFirst();
         recordRejection("pending camera pose overflow");
      }
      incomingVIOCameraPoses.addLast(new IncomingCameraPose(exposureTimestamp, trackedCameraPoseInVIOWorld, leftCameraOffset));
      processIncomingCameraPoses();
   }

   /**
    * Takes pending camera poses off the queue, oldest first, once a robot state newer than their exposure time has arrived, so the
    * robot state at the exposure time can be interpolated between the states before and after it instead of extrapolated.
    */
   private void processIncomingCameraPoses()
   {
      while (!incomingVIOCameraPoses.isEmpty())
      {
         if (incomingVIOCameraPoses.peekFirst().exposureTimestamp >= latestRobotStateTimestamp)
         {
            statusReason = "waiting for a robot state after the exposure";
            return;
         }
         IncomingCameraPose incomingCameraPose = incomingVIOCameraPoses.removeFirst();

         RigidBodyTransform controllerRootPoseAtExposure = new RigidBodyTransform();
         RigidBodyTransform controllerCameraPoseAtExposure = new RigidBodyTransform();
         boolean recentEnough = latestRobotStateTimestamp - incomingCameraPose.exposureTimestamp <= MAXIMUM_EXPOSURE_AGE_NANOSECONDS;
         if (!recentEnough || !controllerRootPoseHistory.getPoseAtTime(incomingCameraPose.exposureTimestamp, controllerRootPoseAtExposure)
             || !controllerCameraPoseHistory.getPoseAtTime(incomingCameraPose.exposureTimestamp, controllerCameraPoseAtExposure))
         {
            recordRejection("no robot states close enough in time before and after the exposure");
            continue;
         }
         RigidBodyTransform controllerLeftCameraPoseAtExposure = new RigidBodyTransform(controllerCameraPoseAtExposure);
         controllerLeftCameraPoseAtExposure.multiply(incomingCameraPose.leftCameraOffset);

         if (!areWorldFramesAnchoredToOneAnother)
         {
            // The anchor: puts this camera pose exactly on the robot state's camera pose at its exposure time.
            vioWorldToRobotWorld.set(controllerLeftCameraPoseAtExposure);
            vioWorldToRobotWorld.multiplyInvertOther(incomingCameraPose.trackedCameraPoseInVIOWorld);
            areWorldFramesAnchoredToOneAnother = true;
         }
         RigidBodyTransformReadOnly vioCameraPoseInControllerWorld = computePoseInControllerWorld(vioWorldToRobotWorld,
                                                                                                  incomingCameraPose.trackedCameraPoseInVIOWorld);
         RigidBodyTransform vioRootPoseInControllerWorld = computeRootPoseFromCameraPose(vioCameraPoseInControllerWorld,
                                                                                         controllerLeftCameraPoseAtExposure,
                                                                                         controllerRootPoseAtExposure);
         // The controller's pose histories are indexed by wall time, so the exposure time is the measurement's timestamp.
         convertedPoseConsumer.accept(incomingCameraPose.exposureTimestamp, vioRootPoseInControllerWorld, vioCameraPoseInControllerWorld);
         acceptedCount++;
         statusReason = "accepted";
      }
   }

   synchronized long getAcceptedCount()
   {
      return acceptedCount;
   }

   synchronized long getRejectedCount()
   {
      return rejectedCount;
   }

   /** Why the last pose was rejected, or what the calculator is waiting for. */
   synchronized String getStatusReason()
   {
      return statusReason;
   }

   synchronized String getStatusDescription()
   {
      return "accepted=%d rejected=%d pending=%d anchored=%s reason=%s".formatted(acceptedCount,
                                                                                  rejectedCount,
                                                                                  incomingVIOCameraPoses.size(),
                                                                                  areWorldFramesAnchoredToOneAnother,
                                                                                  statusReason);
   }

   synchronized void recordRejection(String reason)
   {
      rejectedCount++;
      statusReason = reason;
   }

   /** The root pose implied by a camera pose tracked in the VIO world, given the anchor and the robot state at the exposure. */
   static RigidBodyTransform computeRootPoseFromTrackedCameraPose(RigidBodyTransformReadOnly vioWorldToControllerWorld,
                                                                  RigidBodyTransformReadOnly cameraPoseInVIOWorld,
                                                                  RigidBodyTransformReadOnly robotStateCameraPose,
                                                                  RigidBodyTransformReadOnly robotStateRootPose)
   {
      return computeRootPoseFromCameraPose(computePoseInControllerWorld(vioWorldToControllerWorld, cameraPoseInVIOWorld),
                                           robotStateCameraPose,
                                           robotStateRootPose);
   }

   static RigidBodyTransform computePoseInControllerWorld(RigidBodyTransformReadOnly vioWorldToControllerWorld, RigidBodyTransformReadOnly poseInVIOWorld)
   {
      RigidBodyTransform anchoredCameraPose = new RigidBodyTransform(vioWorldToControllerWorld);
      anchoredCameraPose.multiply(poseInVIOWorld);
      return anchoredCameraPose;
   }

   /** The root pose implied by an anchored camera pose, using the camera-to-root transform from the robot state. */
   private static RigidBodyTransform computeRootPoseFromCameraPose(RigidBodyTransformReadOnly vioCameraPoseInControllerWorld,
                                                                   RigidBodyTransformReadOnly controllerCameraPose,
                                                                   RigidBodyTransformReadOnly controllerRootPose)
   {
      RigidBodyTransform cameraToRootTransform = new RigidBodyTransform(controllerCameraPose);
      cameraToRootTransform.invert();
      cameraToRootTransform.multiply(controllerRootPose);
      RigidBodyTransform vioRootPoseInControllerWorld = new RigidBodyTransform(vioCameraPoseInControllerWorld);
      vioRootPoseInControllerWorld.multiply(cameraToRootTransform);
      return vioRootPoseInControllerWorld;
   }
}

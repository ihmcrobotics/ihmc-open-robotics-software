package us.ihmc.perception.vio;

import controller_msgs.RobotConfigurationData;
import ihmc_common_msgs.ExternalPoseSourceStatus;
import ihmc_common_msgs.StampedPosePacket;
import us.ihmc.communication.ExternalPoseCorrectionAPI;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

import java.util.function.Function;

/**
 * Turns a camera's VIO tracking into pelvis pose measurements for the state estimator in the controller process. It runs in the
 * process that owns the camera.
 * <p>
 * Each robot state the controller publishes is passed to {@link #addRobotState(RobotConfigurationData)}, which computes the
 * pelvis and camera poses from it with forward kinematics on a robot model of its own. {@link VIORootPoseCalculator} pairs each camera pose with the robot
 * state at its exposure time, anchors the VIO world to the robot's world at each session start, and converts camera poses to
 * pelvis poses.
 * </p>
 * <p>
 * Published on the autonomy process's node, all scoped by robot name:
 * <ul>
 * <li>Every converted pelvis pose, as a {@link StampedPosePacket} on the standard state estimator topic. The timestamp is the
 * exposure time and the frame ID is the VIO session. The controller checks each one with its {@code StampedPosePacketValidator},
 * and its pelvis pose correction mode decides whether, and how, it uses them.</li>
 * <li>The anchored camera pose, on {@link ExternalPoseCorrectionAPI#getSensorPoseTopic(String)}, for the operator UI.</li>
 * <li>Session status, on {@link ExternalPoseCorrectionAPI#getSourceStatusTopic(String)}.</li>
 * </ul>
 * VIO starts as soon as the camera is ready, and restarts with a new session on each message on
 * {@link ExternalPoseCorrectionAPI#getResetSourceTopic(String)}.
 * </p>
 */
public class VIOPelvisPoseProvider implements AutoCloseable
{
   /** Robot states and camera poses further than this from the current wall time are rejected as stale. */
   private static final long MAXIMUM_INPUT_AGE_NANOSECONDS = 200_000_000L;
   private static final long STATUS_PUBLISH_PERIOD_NANOSECONDS = 500_000_000L;
   private static final long STATUS_LOG_PERIOD_NANOSECONDS = 5_000_000_000L;

   private final ROS2Node ros2Node;
   private final ROS2Publisher<StampedPosePacket> pelvisPosePublisher;
   private final ROS2Publisher<StampedPosePacket> cameraPosePublisher;
   private final ROS2Publisher<ExternalPoseSourceStatus> statusPublisher;
   private final ROS2Subscription<std_msgs.Empty> resetSubscription;
   private final VIOTrackingSession trackingSession = new VIOTrackingSession();
   private final VIORootPoseCalculator vioRootPoseCalculator;
   private final FullHumanoidRobotModel robotStateModel;
   private final HumanoidReferenceFrames robotStateReferenceFrames;
   private final ReferenceFrame robotStateCameraFrame;
   private final OneDoFJointBasics[] robotStateJoints;
   private final int expectedJointNameHash;
   private boolean initialTrackingStartRequested;
   private String trackingRestartFailure = "";
   private long lastStatusPublishTime;
   private long lastStatusLogTime;
   private boolean closed;

   /**
    * @param robotName           the robot name the topics are scoped by.
    * @param robotStateModel     a robot model for this provider only, which it sets from each robot state.
    * @param sensorInformation   the robot's sensor information, for its reference frames.
    * @param cameraFrameSelector selects the tracking camera's mounting frame from the robot's reference frames.
    * @param ros2Node            the node to publish and subscribe on.
    */
   public VIOPelvisPoseProvider(String robotName,
                                FullHumanoidRobotModel robotStateModel,
                                HumanoidRobotSensorInformation sensorInformation,
                                Function<HumanoidReferenceFrames, ReferenceFrame> cameraFrameSelector,
                                ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
      this.robotStateModel = robotStateModel;
      robotStateReferenceFrames = new HumanoidReferenceFrames(robotStateModel, sensorInformation);
      robotStateCameraFrame = cameraFrameSelector.apply(robotStateReferenceFrames);
      // RobotConfigurationData carries the full ordered joint list, including the hand joints.
      robotStateJoints = robotStateModel.getOneDoFJoints();
      expectedJointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(robotStateJoints,
                                                                                   robotStateModel.getForceSensorDefinitions(),
                                                                                   robotStateModel.getIMUDefinitions());

      pelvisPosePublisher = ros2Node.createPublisher(StateEstimatorAPI.getTopic(StampedPosePacket.class, robotName));
      cameraPosePublisher = ros2Node.createPublisher(ExternalPoseCorrectionAPI.getSensorPoseTopic(robotName));
      statusPublisher = ros2Node.createPublisher(ExternalPoseCorrectionAPI.getSourceStatusTopic(robotName));
      // Every converted pose is published; the controller decides what to use. The calculator's validity check isn't used here.
      vioRootPoseCalculator = new VIORootPoseCalculator(this::publishVIOPosesInControllerFrame);

      resetSubscription = ros2Node.createSubscriptionSampler(ExternalPoseCorrectionAPI.getResetSourceTopic(robotName),
                                                             resetMessage -> requestTrackingRestart());

      LogTools.info("VIO pelvis pose provider started. Requires synchronized camera and controller wall clocks and a calibrated camera mounting.");
   }

   /** Publishes every converted pose: the root pose for the controller, and the camera pose for the operator UI. */
   private void publishVIOPosesInControllerFrame(long exposureTimestamp,
                                                 RigidBodyTransformReadOnly rootPoseInControllerWorld,
                                                 RigidBodyTransformReadOnly cameraPoseInControllerWorld)
   {
      trackingSession.markAnchored();
      publishStampedPose(pelvisPosePublisher, exposureTimestamp, rootPoseInControllerWorld);
      publishStampedPose(cameraPosePublisher, exposureTimestamp, cameraPoseInControllerWorld);
   }

   private void publishStampedPose(ROS2Publisher<StampedPosePacket> publisher, long exposureTimestamp, RigidBodyTransformReadOnly pose)
   {
      StampedPosePacket stampedPose = new StampedPosePacket();
      stampedPose.setTimestamp(exposureTimestamp);
      stampedPose.setConfidenceFactor(1.0);
      stampedPose.getFrameId().append(trackingSession.getSessionId());
      stampedPose.getPose().set(pose);
      publisher.publish(stampedPose);
   }

   private synchronized void requestTrackingRestart()
   {
      if (closed)
         return;
      initialTrackingStartRequested = true;
      trackingRestartFailure = "";
      trackingSession.requestTrackingRestart(getCurrentWallTime());
      vioRootPoseCalculator.resetForNewSession();
      publishStatus();
   }

   /** Polled only by the camera's grab thread, so no camera SDK calls are made from ROS callbacks. */
   public synchronized long takePendingTrackingRestartRequest()
   {
      if (closed)
         return 0;
      // Start once the camera is ready: opening the camera can take a while.
      if (!initialTrackingStartRequested)
         requestTrackingRestart();
      return trackingSession.takePendingRestartRequest();
   }

   /** On the camera grab thread, after it restarted tracking for {@code requestToken}. */
   public synchronized void finishTrackingRestart(long requestToken, boolean restartSucceeded)
   {
      if (closed || !trackingSession.finishTrackingRestart(requestToken, restartSucceeded, getCurrentWallTime()))
         return;
      if (!restartSucceeded)
         trackingRestartFailure = "Camera tracking restart failed; use Start / Reset VIO to retry";
      publishStatus();
   }

   /** On the camera grab thread, when the camera is about to reopen. Stops the session until the next Start / Reset. */
   public synchronized void stopTrackingForCameraReopen()
   {
      if (!closed && trackingSession.stopForCameraReopen())
      {
         vioRootPoseCalculator.resetForNewSession();
         publishStatus();
      }
   }

   /** Called periodically by the autonomy process; publishes status at most once per {@link #STATUS_PUBLISH_PERIOD_NANOSECONDS}. */
   public synchronized void publishStatusIfDue()
   {
      if (!closed && System.nanoTime() - lastStatusPublishTime >= STATUS_PUBLISH_PERIOD_NANOSECONDS)
         publishStatus();
   }

   private void publishStatus()
   {
      ExternalPoseSourceStatus status = new ExternalPoseSourceStatus();
      status.setSessionId(trackingSession.getSessionId());
      status.setSessionStartTime(trackingSession.getSessionStartTime());
      status.setState(trackingSession.getPhase().name());
      status.setAcceptedCount(vioRootPoseCalculator.getAcceptedCount());
      status.setRejectedCount(vioRootPoseCalculator.getRejectedCount());
      status.setDetail(trackingRestartFailure.isEmpty() ? vioRootPoseCalculator.getStatusDescription() : trackingRestartFailure);
      statusPublisher.publish(status);
      lastStatusPublishTime = System.nanoTime();
   }

   /** A robot state published by the controller: computes the pelvis and camera poses in the robot's world. */
   public synchronized void addRobotState(RobotConfigurationData robotConfigurationData)
   {
      if (closed || !trackingSession.isTracking())
         return;
      if (robotConfigurationData.getJointNameHash() != expectedJointNameHash || robotConfigurationData.getJointAngles().size() != robotStateJoints.length)
      {
         vioRootPoseCalculator.recordRejection("robot joint hash/size mismatch expectedHash=%d actualHash=%d expectedJoints=%d actualJoints=%d".formatted(
               expectedJointNameHash,
               robotConfigurationData.getJointNameHash(),
               robotStateJoints.length,
               robotConfigurationData.getJointAngles().size()));
         logStatusIfDue();
         return;
      }
      long robotStateTimestamp = robotConfigurationData.getWallTime();
      if (!trackingSession.acceptsDataAt(robotStateTimestamp))
         return;
      if (Math.abs(getCurrentWallTime() - robotStateTimestamp) > MAXIMUM_INPUT_AGE_NANOSECONDS)
      {
         vioRootPoseCalculator.recordRejection("stale robot state or unsynchronized wall clocks");
         logStatusIfDue();
         return;
      }
      for (int jointIndex = 0; jointIndex < robotStateJoints.length; jointIndex++)
      {
         if (!Float.isFinite(robotConfigurationData.getJointAngles().get(jointIndex)))
         {
            vioRootPoseCalculator.recordRejection("nonfinite joint state");
            return;
         }
      }
      robotStateModel.getRootJoint()
                     .getJointPose()
                     .set(robotConfigurationData.getRootPosition().getPoint(), robotConfigurationData.getRootOrientation().getQuaternion());
      for (int jointIndex = 0; jointIndex < robotStateJoints.length; jointIndex++)
         robotStateJoints[jointIndex].setQ(robotConfigurationData.getJointAngles().get(jointIndex));
      robotStateReferenceFrames.updateFrames();
      vioRootPoseCalculator.addRobotControllerState(robotStateTimestamp,
                                                    robotStateModel.getRootJoint().getJointPose(),
                                                    robotStateCameraFrame.getTransformToWorldFrame());
      logStatusIfDue();
   }

   /**
    * On the camera grab thread: the camera's tracking pose for this grab, in the VIO world. The exposure timestamp is epoch
    * nanoseconds, and the pose is the left camera's, in FLU. {@code null} means there's no valid tracking for this grab.
    *
    * @param leftCameraOffset the transform from the camera's mounting frame to the frame the tracking pose is reported in.
    */
   public synchronized void addCameraTrackingPose(long exposureTimestamp,
                                                  RigidBodyTransformReadOnly trackedCameraPoseInVIOWorld,
                                                  RigidBodyTransformReadOnly leftCameraOffset)
   {
      if (closed || !trackingSession.acceptsDataAt(exposureTimestamp))
         return;
      if (Math.abs(getCurrentWallTime() - exposureTimestamp) > MAXIMUM_INPUT_AGE_NANOSECONDS)
      {
         vioRootPoseCalculator.recordRejection("stale image or unsynchronized camera wall clock");
      }
      else if (trackedCameraPoseInVIOWorld != null && !trackedCameraPoseInVIOWorld.containsNaN())
      {
         trackingSession.markTrackingValid();
         vioRootPoseCalculator.addCameraTrackingPose(exposureTimestamp, trackedCameraPoseInVIOWorld, leftCameraOffset);
      }
      else
      {
         vioRootPoseCalculator.recordRejection("waiting for valid camera tracking");
      }
      logStatusIfDue();
   }

   private void logStatusIfDue()
   {
      if (System.nanoTime() - lastStatusLogTime > STATUS_LOG_PERIOD_NANOSECONDS)
      {
         lastStatusLogTime = System.nanoTime();
         LogTools.info("VIO pelvis pose provider {}", vioRootPoseCalculator.getStatusDescription());
      }
   }

   private static long getCurrentWallTime()
   {
      return System.currentTimeMillis() * 1_000_000L;
   }

   /** Stops handling robot states and camera poses, and removes this provider's publishers and subscription from the node. */
   @Override
   public void close()
   {
      synchronized (this)
      {
         closed = true;
      }
      ros2Node.destroySubscription(resetSubscription);
      ros2Node.destroyPublisher(pelvisPosePublisher);
      ros2Node.destroyPublisher(cameraPosePublisher);
      ros2Node.destroyPublisher(statusPublisher);
   }
}

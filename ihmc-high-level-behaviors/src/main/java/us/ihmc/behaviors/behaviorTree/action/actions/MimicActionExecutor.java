package us.ihmc.behaviors.behaviorTree.action.actions;

import controller_msgs.HighLevelStateMessage;
import toolbox_msgs.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.MimicActionDefinition.MimicActionType;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ros2log.ROS2LogReplay;
import us.ihmc.communication.ros2log.ROS2LogTimeSource;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.NonWallTimer;

import java.io.File;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

public class MimicActionExecutor extends ActionNodeExecutor<MimicActionState, MimicActionDefinition>
{
   private static final File DEFAULT_ROS2_LOG_DIRECTORY = IHMCCommonPaths.LOGS_DIRECTORY.resolve("ros2").toFile();
   private static final String ROS2_LOG_DIRECTORY_MARKER = "/.ihmc/logs/ros2/";

   private final ROS2LogReplay ros2Replayer;
   private final ROS2Topic<KinematicsToolboxOutputStatus> kstOutputTopic;
   private final NonWallTimer transitionTimer = new NonWallTimer();
   private final FullHumanoidRobotModel ghostFullRobotModel;
   private final OneDoFJointBasics[] ghostOneDoFJoints;
   private final HumanoidReferenceFrames ghostReferenceFrames;
   private String loadedMimicFileName = "";
   private volatile boolean replayThreadRunning = false;
   private volatile boolean replayCompleted = false;
   private volatile boolean replayFailed = false;
   private Thread replayThread;
   private boolean transitionRequestSent = false;
   private boolean replayAlignmentCaptured = false;
   private final Object replayAlignmentLock = new Object();

   /**
    * Frozen/live target: robot pelvis XY/yaw and mid-feet Z used to place the replay.
    * While next-for-execution this tracks the live robot; once execution starts it is frozen.
    */
   private boolean alignmentTargetFrozen = false;
   private final Point2D targetPelvisXY = new Point2D();
   private double targetPelvisYaw = 0.0;
   private double targetMidFeetZ = 0.0;

   /** Untransformed first-frame ghost pelvis XY/yaw and mid-feet Z (from FK on the logged pose). */
   private boolean replayBaselineInitialized = false;
   private final Point2D baselineGhostPelvisXY = new Point2D();
   private double baselineGhostPelvisYaw = 0.0;
   private double baselineGhostMidFeetZ = 0.0;

   private final FramePose3D tempPose = new FramePose3D();
   private final Point3D tempPosition = new Point3D();
   private final Quaternion tempOrientation = new Quaternion();
   private final Map<KinematicsToolboxOutputStatus, ReplayStatusBaseline> replayStatusBaselines = new IdentityHashMap<>();
   private boolean holdingFirstFrame = false;
   private boolean holdingLastFrame = false;

   public MimicActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new MimicActionState(id, rootNode.getState()), rootNode);

      List<ROS2Topic<?>> topics = new ArrayList<>();
      { // KST output
         kstOutputTopic = KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName());
         topics.add(kstOutputTopic);
      }
      ROS2LogTimeSource timeSource = ROS2LogTimeSource.SYSTEM;
      ros2Replayer = new ROS2LogReplay(robotModel.getSimpleRobotName(), topics, timeSource);

      ghostFullRobotModel = robotModel.createFullRobotModel();
      ghostOneDoFJoints = ghostFullRobotModel.getOneDoFJoints();
      ghostReferenceFrames = new HumanoidReferenceFrames(ghostFullRobotModel, robotModel.getSensorInformation());
   }

   @Override
   public void update()
   {
      super.update();
      transitionTimer.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      if (definition.getMimicActionType().getValue() != MimicActionType.EXECUTE_POLICY)
      {
         clearFrameHold();
         if (!state.getIsExecuting())
            stopReplayThread();
         return;
      }

      ensureMimicLogLoaded();

      if (state.getIsExecuting())
      {
         // Active playback is handled in updateCurrentlyExecuting / replay thread.
         holdingFirstFrame = false;
         holdingLastFrame = false;
         return;
      }

      stopReplayThread();

      if (state.getIsNextForExecution())
      {
         // Preview first frame aligned to the live robot pose.
         alignmentTargetFrozen = false;
         updateAlignmentTargetFromRobot();
         holdFirstFrameIfNeeded();
         republishHeldFrame();
      }
      else if (holdingLastFrame && !isAnotherLeafExecuting())
      {
         republishHeldFrame();
      }
      else
      {
         clearFrameHold();
      }
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      replayCompleted = false;
      replayFailed = false;
      transitionRequestSent = false;
      replayAlignmentCaptured = false;
      holdingFirstFrame = false;
      holdingLastFrame = false;
      stopReplayThread();
      // Keep pristine message baselines; only freeze the robot-side alignment target at execute time.
      synchronized (replayAlignmentLock)
      {
         alignmentTargetFrozen = false;
      }

      if (definition.getMimicActionType().getValue() == MimicActionType.EXECUTE_POLICY)
      {
         ensureMimicLogLoaded();
         ros2Replayer.reset();
         sendStateTransitionRequest(true);
         transitionRequestSent = true;
      }
      else if (definition.getMimicActionType().getValue() == MimicActionType.EXIT_POLICY)
      {
         transitionTimer.reset();
         sendStateTransitionRequest(false);
         transitionRequestSent = true;
      }
   }

   private void ensureMimicLogLoaded()
   {
      String mimicFileName = definition.getMimicFileName();
      if (mimicFileName == null || mimicFileName.isBlank())
         return;

      if (!mimicFileName.equals(loadedMimicFileName))
      {
         ros2Replayer.load(resolveMimicLogFile(mimicFileName));
         loadedMimicFileName = mimicFileName;
         holdingFirstFrame = false;
         holdingLastFrame = false;
         synchronized (replayAlignmentLock)
         {
            replayStatusBaselines.clear();
            replayBaselineInitialized = false;
         }
         LogTools.info("Loaded mimic file: {}", mimicFileName);
      }

      if (ros2Replayer.isReady())
         ros2Replayer.addReplayMutator(kstOutputTopic, (status, timestamp) -> alignReplayStatusToActionStart(status));
   }

   private void holdFirstFrameIfNeeded()
   {
      if (holdingFirstFrame || !ros2Replayer.isReady())
         return;

      ros2Replayer.holdFirstFrame();
      holdingFirstFrame = true;
      holdingLastFrame = false;
   }

   private void holdLastFrame()
   {
      if (!ros2Replayer.isReady())
         return;

      ros2Replayer.holdLastFrame();
      holdingLastFrame = true;
      holdingFirstFrame = false;
   }

   private void republishHeldFrame()
   {
      if (ros2Replayer.isReady())
         ros2Replayer.doIncrementalReplay();
   }

   private void clearFrameHold()
   {
      holdingFirstFrame = false;
      holdingLastFrame = false;
   }

   private boolean isAnotherLeafExecuting()
   {
      for (LeafNodeState<?> leafState : rootNode.getState().getOrderedLeaves())
      {
         if (leafState.getID() != state.getID() && leafState.getIsExecuting())
            return true;
      }
      return false;
   }

   private void captureActionStartAlignment()
   {
      synchronized (replayAlignmentLock)
      {
         updateAlignmentTargetFromRobot();
         alignmentTargetFrozen = true;
      }
   }

   private void updateAlignmentTargetFromRobot()
   {
      var pelvisPose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getPelvisZUpFrame);
      targetPelvisXY.set(pelvisPose.getPosition().getX(), pelvisPose.getPosition().getY());
      targetPelvisYaw = pelvisPose.getYaw();

      var midFeetPose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame);
      targetMidFeetZ = midFeetPose.getPosition().getZ();
   }

   private void alignReplayStatusToActionStart(KinematicsToolboxOutputStatus status)
   {
      synchronized (replayAlignmentLock)
      {
         ReplayStatusBaseline baseline = replayStatusBaselines.get(status);
         if (baseline == null)
         {
            // Capture pristine logged values before any transform is applied.
            baseline = new ReplayStatusBaseline(status);
            replayStatusBaselines.put(status, baseline);
         }

         if (!replayBaselineInitialized)
            initializeReplayBaselineFromStatus(baseline);

         if (!alignmentTargetFrozen)
            updateAlignmentTargetFromRobot();

         double yawOffset = targetPelvisYaw - baselineGhostPelvisYaw;
         double zOffset = targetMidFeetZ - baselineGhostMidFeetZ;
         double cosYaw = Math.cos(yawOffset);
         double sinYaw = Math.sin(yawOffset);

         applyPlanarAlignment(status.getDesiredRootPosition().getPoint(),
                              status.getDesiredRootOrientation().getQuaternion(),
                              baseline.pelvisX,
                              baseline.pelvisY,
                              baseline.pelvisZ,
                              baseline.pelvisYaw,
                              baseline.pelvisPitch,
                              baseline.pelvisRoll,
                              yawOffset,
                              zOffset,
                              cosYaw,
                              sinYaw);
         applyPlanarAlignment(status.getDesiredTorsoPosition().getPoint(),
                              status.getDesiredTorsoOrientation().getQuaternion(),
                              baseline.torsoX,
                              baseline.torsoY,
                              baseline.torsoZ,
                              baseline.torsoYaw,
                              baseline.torsoPitch,
                              baseline.torsoRoll,
                              yawOffset,
                              zOffset,
                              cosYaw,
                              sinYaw);
      }
   }

   private void initializeReplayBaselineFromStatus(ReplayStatusBaseline baseline)
   {
      updateGhostFromBaseline(baseline);

      tempPose.setFromReferenceFrame(ghostReferenceFrames.getPelvisZUpFrame());
      baselineGhostPelvisXY.set(tempPose.getPosition().getX(), tempPose.getPosition().getY());
      baselineGhostPelvisYaw = tempPose.getYaw();

      tempPose.setFromReferenceFrame(ghostReferenceFrames.getMidFeetZUpFrame());
      baselineGhostMidFeetZ = tempPose.getPosition().getZ();

      replayBaselineInitialized = true;
   }

   private void applyPlanarAlignment(us.ihmc.euclid.tuple3D.interfaces.Point3DBasics positionToPack,
                                     us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics orientationToPack,
                                     double baselineX,
                                     double baselineY,
                                     double baselineZ,
                                     double baselineYaw,
                                     double baselinePitch,
                                     double baselineRoll,
                                     double yawOffset,
                                     double zOffset,
                                     double cosYaw,
                                     double sinYaw)
   {
      double relX = baselineX - baselineGhostPelvisXY.getX();
      double relY = baselineY - baselineGhostPelvisXY.getY();
      double rotatedX = cosYaw * relX - sinYaw * relY;
      double rotatedY = sinYaw * relX + cosYaw * relY;

      positionToPack.setX(rotatedX + targetPelvisXY.getX());
      positionToPack.setY(rotatedY + targetPelvisXY.getY());
      positionToPack.setZ(baselineZ + zOffset);

      orientationToPack.setYawPitchRoll(baselineYaw + yawOffset, baselinePitch, baselineRoll);
   }

   private void updateGhostFromBaseline(ReplayStatusBaseline baseline)
   {
      tempPosition.set(baseline.pelvisX, baseline.pelvisY, baseline.pelvisZ);
      tempOrientation.setYawPitchRoll(baseline.pelvisYaw, baseline.pelvisPitch, baseline.pelvisRoll);
      ghostFullRobotModel.getRootJoint().setJointPosition(tempPosition);
      ghostFullRobotModel.getRootJoint().setJointOrientation(tempOrientation);

      for (int i = 0; i < ghostOneDoFJoints.length && i < baseline.jointAngles.length; i++)
         ghostOneDoFJoints[i].setQ(baseline.jointAngles[i]);

      ghostFullRobotModel.getElevator().updateFramesRecursively();
      ghostReferenceFrames.updateFrames();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      switch (definition.getMimicActionType().getValue())
      {
         case EXECUTE_POLICY ->
         {
            state.setNominalExecutionDuration(0.0);
            state.setElapsedExecutionTime(0.0);

            HighLevelControllerName latestControllerState = controllerStatusTracker.getLatestKnownState();
            if (latestControllerState != HighLevelControllerName.RL_CONTROL)
            {
               if (!transitionRequestSent)
               {
                  sendStateTransitionRequest(true);
                  transitionRequestSent = true;
               }
               return;
            }

            if (!replayAlignmentCaptured)
            {
               captureActionStartAlignment();
               replayAlignmentCaptured = true;
            }

            if (!ros2Replayer.isReady())
            {
               state.getLogger().error("Mimic replay is not ready. File: {}", definition.getMimicFileName());
               state.setFailed(true);
               state.setIsExecuting(false);
               return;
            }

            if (replayFailed)
            {
               state.getLogger().error("Mimic replay thread failed. File: {}", definition.getMimicFileName());
               state.setFailed(true);
               state.setIsExecuting(false);
               stopReplayThread();
               return;
            }

            if (replayCompleted)
            {
               state.setIsExecuting(false);
               stopReplayThread();
               holdLastFrame();
               return;
            }

            if (!replayThreadRunning)
               startReplayThread();
         }
         case EXIT_POLICY ->
         {
            stopReplayThread();
            clearFrameHold();
            state.setNominalExecutionDuration(definition.getWaitTimeExitPolicy());
            state.setElapsedExecutionTime(transitionTimer.getElapsedTime());

            HighLevelControllerName latestControllerState = controllerStatusTracker.getLatestKnownState();
            boolean reachedWalking = latestControllerState == HighLevelControllerName.WALKING;
            boolean durationElapsed = !transitionTimer.isRunning(definition.getWaitTimeExitPolicy());

            if (reachedWalking && durationElapsed)
            {
               state.setIsExecuting(false);
               return;
            }

            state.setIsExecuting(true);

            if (!transitionRequestSent)
            {
               sendStateTransitionRequest(false);
               transitionRequestSent = true;
            }
         }
      }
   }

   private void startReplayThread()
   {
      if (replayThreadRunning)
         return;

      replayThreadRunning = true;
      replayThread = new Thread(() ->
      {
         final long intervalNanos = 1_000_000L; // 1 kHz
         while (replayThreadRunning)
         {
            long start = System.nanoTime();
            try
            {
               if (ros2Replayer.isReady() && ros2Replayer.doIncrementalReplay())
               {
                  replayCompleted = true;
                  replayThreadRunning = false;
                  break;
               }
            }
            catch (Exception e)
            {
               replayFailed = true;
               replayThreadRunning = false;
            }

            long elapsed = System.nanoTime() - start;
            long remaining = intervalNanos - elapsed;
            if (remaining > 0)
            {
               try
               {
                  Thread.sleep(remaining / 1_000_000, (int) (remaining % 1_000_000));
               }
               catch (InterruptedException ignored)
               {
               }
            }
         }
      }, "MimicActionReplay-" + state.getID());
      replayThread.setDaemon(true);
      replayThread.start();
   }

   private void stopReplayThread()
   {
      replayThreadRunning = false;
      if (replayThread != null)
      {
         replayThread.interrupt();
         replayThread = null;
      }
   }

   private void sendStateTransitionRequest(boolean activate)
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      if (activate)
         highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.RL_CONTROL.toByte());
      else
         highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.WALKING.toByte());
      ros2ControllerHelper.publishToController(highLevelStateMessage);
   }

   private File resolveMimicLogFile(String storedPath)
   {
      File asFile = new File(storedPath);
      if (!asFile.isAbsolute())
         return new File(DEFAULT_ROS2_LOG_DIRECTORY, storedPath);

      String normalized = storedPath.replace('\\', '/');
      int markerIndex = normalized.indexOf(ROS2_LOG_DIRECTORY_MARKER);
      if (markerIndex >= 0)
      {
         String suffix = normalized.substring(markerIndex + ROS2_LOG_DIRECTORY_MARKER.length());
         return new File(DEFAULT_ROS2_LOG_DIRECTORY, suffix);
      }

      return asFile;
   }

   private static class ReplayStatusBaseline
   {
      private final double pelvisX;
      private final double pelvisY;
      private final double pelvisZ;
      private final double torsoX;
      private final double torsoY;
      private final double torsoZ;
      private final double pelvisYaw;
      private final double pelvisPitch;
      private final double pelvisRoll;
      private final double torsoYaw;
      private final double torsoPitch;
      private final double torsoRoll;
      private final double[] jointAngles;

      private ReplayStatusBaseline(KinematicsToolboxOutputStatus status)
      {
         pelvisX = status.getDesiredRootPosition().getPoint().getX();
         pelvisY = status.getDesiredRootPosition().getPoint().getY();
         pelvisZ = status.getDesiredRootPosition().getPoint().getZ();
         torsoX = status.getDesiredTorsoPosition().getPoint().getX();
         torsoY = status.getDesiredTorsoPosition().getPoint().getY();
         torsoZ = status.getDesiredTorsoPosition().getPoint().getZ();
         pelvisYaw = status.getDesiredRootOrientation().getQuaternion().getYaw();
         pelvisPitch = status.getDesiredRootOrientation().getQuaternion().getPitch();
         pelvisRoll = status.getDesiredRootOrientation().getQuaternion().getRoll();
         torsoYaw = status.getDesiredTorsoOrientation().getQuaternion().getYaw();
         torsoPitch = status.getDesiredTorsoOrientation().getQuaternion().getPitch();
         torsoRoll = status.getDesiredTorsoOrientation().getQuaternion().getRoll();

         jointAngles = new double[status.getDesiredJointAngles().size()];
         for (int i = 0; i < jointAngles.length; i++)
            jointAngles[i] = status.getDesiredJointAngles().get(i);
      }
   }
}

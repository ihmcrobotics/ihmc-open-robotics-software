package us.ihmc.behaviors.behaviorTree.action.actions;

import controller_msgs.HighLevelStateMessage;
import toolbox_msgs.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.MimicActionDefinition.MimicActionType;
import us.ihmc.communication.ros2log.ROS2LogReplay;
import us.ihmc.communication.ros2log.ROS2LogTimeSource;
import us.ihmc.commons.Conversions;
import us.ihmc.tools.NonWallTimer;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class MimicActionExecutor extends ActionNodeExecutor<MimicActionState, MimicActionDefinition>
{
   private static final File DEFAULT_ROS2_LOG_DIRECTORY = new File(System.getProperty("user.home"), ".ihmc/logs/ros2");

   private final ROS2LogReplay ros2Replayer;
   private final ROS2Topic<KinematicsToolboxOutputStatus> kstOutputTopic;
   private final NonWallTimer transitionTimer = new NonWallTimer();
   private String loadedMimicFileName = "";
   private volatile boolean replayThreadRunning = false;
   private volatile boolean replayCompleted = false;
   private volatile boolean replayFailed = false;
   private Thread replayThread;
   private boolean transitionRequestSent = false;
   private final Object replayAlignmentLock = new Object();
   private boolean replayAlignmentInitialized = false;
   private final Point2D actionStartPelvisPosition = new Point2D();
   private final Point2D actionStartTorsoPosition = new Point2D();
   private double actionStartPelvisYaw = 0.0;
   private double actionStartTorsoYaw = 0.0;
   private final Vector2D replayPelvisPositionOffset = new Vector2D();
   private final Vector2D replayTorsoPositionOffset = new Vector2D();
   private double replayPelvisYawOffset = 0.0;
   private double replayTorsoYawOffset = 0.0;

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
   }

   @Override
   public void update()
   {
      super.update();
      transitionTimer.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      if (!state.getIsExecuting())
         stopReplayThread();
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      replayCompleted = false;
      replayFailed = false;
      transitionRequestSent = false;
      stopReplayThread();

      if (definition.getMimicActionType().getValue() == MimicActionType.EXECUTE_POLICY)
      {
         captureActionStartAlignment();
         String mimicFileName = definition.getMimicFileName();
         if (!mimicFileName.equals(loadedMimicFileName))
         {
            ros2Replayer.load(new File(DEFAULT_ROS2_LOG_DIRECTORY, mimicFileName));
            loadedMimicFileName = mimicFileName;
            LogTools.info("Loaded mimic file: {}", mimicFileName);
         }
         if (ros2Replayer.isReady())
            ros2Replayer.addReplayMutator(kstOutputTopic, (status, timestamp) -> alignReplayStatusToActionStart(status));
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

   private void captureActionStartAlignment()
   {
      synchronized (replayAlignmentLock)
      {
         var pelvisPose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getPelvisFrame);
         actionStartPelvisPosition.set(pelvisPose.getPosition().getX(), pelvisPose.getPosition().getY());
         actionStartPelvisYaw = pelvisPose.getYaw();

         var torsoPose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getChestFrame);
         actionStartTorsoPosition.set(torsoPose.getPosition().getX(), torsoPose.getPosition().getY());
         actionStartTorsoYaw = torsoPose.getYaw();

         replayPelvisPositionOffset.setToZero();
         replayTorsoPositionOffset.setToZero();
         replayPelvisYawOffset = 0.0;
         replayTorsoYawOffset = 0.0;
         replayAlignmentInitialized = false;
      }
   }

   private void alignReplayStatusToActionStart(KinematicsToolboxOutputStatus status)
   {
      synchronized (replayAlignmentLock)
      {
         if (!replayAlignmentInitialized)
         {
            replayPelvisPositionOffset.set(actionStartPelvisPosition.getX() - status.getDesiredRootPosition().getPoint().getX(),
                                           actionStartPelvisPosition.getY() - status.getDesiredRootPosition().getPoint().getY());
            replayTorsoPositionOffset.set(actionStartTorsoPosition.getX() - status.getDesiredTorsoPosition().getPoint().getX(),
                                          actionStartTorsoPosition.getY() - status.getDesiredTorsoPosition().getPoint().getY());
            replayPelvisYawOffset = actionStartPelvisYaw - status.getDesiredRootOrientation().getQuaternion().getYaw();
            replayTorsoYawOffset = actionStartTorsoYaw - status.getDesiredTorsoOrientation().getQuaternion().getYaw();
            replayAlignmentInitialized = true;
         }

         status.getDesiredRootPosition().getPoint().setX(status.getDesiredRootPosition().getPoint().getX() + replayPelvisPositionOffset.getX());
         status.getDesiredRootPosition().getPoint().setY(status.getDesiredRootPosition().getPoint().getY() + replayPelvisPositionOffset.getY());
         status.getDesiredTorsoPosition().getPoint().setX(status.getDesiredTorsoPosition().getPoint().getX() + replayTorsoPositionOffset.getX());
         status.getDesiredTorsoPosition().getPoint().setY(status.getDesiredTorsoPosition().getPoint().getY() + replayTorsoPositionOffset.getY());

         status.getDesiredRootOrientation().getQuaternion().setYawPitchRoll(status.getDesiredRootOrientation().getQuaternion().getYaw() + replayPelvisYawOffset,
                                                                            status.getDesiredRootOrientation().getQuaternion().getPitch(),
                                                                            status.getDesiredRootOrientation().getQuaternion().getRoll());
         status.getDesiredTorsoOrientation().getQuaternion().setYawPitchRoll(status.getDesiredTorsoOrientation().getQuaternion().getYaw() + replayTorsoYawOffset,
                                                                               status.getDesiredTorsoOrientation().getQuaternion().getPitch(),
                                                                               status.getDesiredTorsoOrientation().getQuaternion().getRoll());
      }
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
               return;
            }

            if (!replayThreadRunning)
               startReplayThread();
         }
         case EXIT_POLICY ->
         {
            stopReplayThread();
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
}

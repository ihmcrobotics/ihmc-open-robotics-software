package us.ihmc.behaviors.behaviorTree.action.actions;

import controller_msgs.msg.dds.HighLevelStateMessage;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.actions.MimicActionDefinition.MimicActionType;
import us.ihmc.communication.ros2log.ROS2LogReplay;
import us.ihmc.communication.ros2log.ROS2LogTimeSource;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Topic;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class MimicActionExecutor extends ActionNodeExecutor<MimicActionState, MimicActionDefinition>
{
   private static final File DEFAULT_ROS2_LOG_DIRECTORY = new File(System.getProperty("user.home"), ".ihmc/logs/ros2");

   private final ROS2LogReplay ros2Replayer;
   private String loadedMimicFileName = "";
   private volatile boolean replayThreadRunning = false;
   private volatile boolean replayCompleted = false;
   private volatile boolean replayFailed = false;
   private Thread replayThread;

   public MimicActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new MimicActionState(id, rootNode.getState()), rootNode);

      List<ROS2Topic<?>> topics = new ArrayList<>();
      { // KST output
         topics.add(KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName()));
      }
      ROS2LogTimeSource timeSource = ROS2LogTimeSource.SYSTEM;
      ros2Replayer = new ROS2LogReplay(robotModel.getSimpleRobotName(), topics, timeSource);
   }

   @Override
   public void update()
   {
      super.update();

      if (!state.getIsExecuting())
         stopReplayThread();
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      replayCompleted = false;
      replayFailed = false;
      stopReplayThread();

      if (definition.getMimicActionType().getValue() == MimicActionType.EXECUTE_POLICY)
      {
         String mimicFileName = definition.getMimicFileName();
         if (!mimicFileName.equals(loadedMimicFileName))
         {
            ros2Replayer.load(new File(DEFAULT_ROS2_LOG_DIRECTORY, mimicFileName));
            loadedMimicFileName = mimicFileName;
            LogTools.info("Loaded mimic file: {}", mimicFileName);
         }
         ros2Replayer.reset();
         startReplayThread();
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      switch (definition.getMimicActionType().getValue())
      {
         case POLICY_TRANSITION ->
         {
            stopReplayThread();
            sendStateTransitionRequest(true);
            state.setIsExecuting(false);
         }
         case EXIT_POLICY ->
         {
            stopReplayThread();
            sendStateTransitionRequest(false);
            state.setIsExecuting(false);
         }
         case EXECUTE_POLICY ->
         {
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

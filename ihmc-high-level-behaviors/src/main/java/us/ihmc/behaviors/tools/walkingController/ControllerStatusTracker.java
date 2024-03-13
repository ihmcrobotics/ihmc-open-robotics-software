package us.ihmc.behaviors.tools.walkingController;

import controller_msgs.msg.dds.*;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogToolsWriteOnly;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.tools.Timer;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.thread.Throttler;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition.getTopic;

/**
 * A class to keep track of the controller by listening to its ROS 2 status API.
 * <p>
 * WalkingControllerFailureStatusMessage - This is reported when robot is falling over
 * TextToSpeechPacket - All it does is say "walking" on walking start
 */
public class ControllerStatusTracker
{
   private static final double CAPTURABILITY_BASED_STATUS_EXPIRATION_TIME = 0.25;

   private final WalkingFootstepTracker footstepTracker;
   private final LogToolsWriteOnly statusLogger;
   private volatile HighLevelControllerName latestKnownState;
   private final Vector3D lastPlanOffset = new Vector3D();
   private final Timer capturabilityBasedStatusTimer = new Timer();
   private final Timer robotConfigurationDataTimer = new Timer();
   private volatile boolean isWalking = false;
   private volatile boolean isWalkingFromConfigurationData = false;
   private final Notification finishedWalkingNotification = new Notification();
   private final ArrayList<Runnable> notWalkingStateAnymoreCallbacks = new ArrayList<>();
   private final Throttler notWalkingStateAnymoreCallbackThrottler = new Throttler();

   private final List<Notification> abortedListeners = new ArrayList<>();
   private CapturabilityBasedStatus latestCapturabilityBasedStatus;

   public ControllerStatusTracker(LogToolsWriteOnly statusLogger, ROS2NodeInterface ros2Node, String robotName)
   {
      this.statusLogger = statusLogger;
      footstepTracker = new WalkingFootstepTracker(ros2Node, robotName);

      finishedWalkingNotification.set();

      new IHMCROS2Callback<>(ros2Node, ROS2Tools.getRobotConfigurationDataTopic(robotName), this::acceptRobotConfigurationData);
      new IHMCROS2Callback<>(ros2Node, getTopic(HighLevelStateChangeStatusMessage.class, robotName), this::acceptHighLevelStateChangeStatusMessage);
      new IHMCROS2Callback<>(ros2Node, getTopic(WalkingControllerFailureStatusMessage.class, robotName), this::acceptWalkingControllerFailureStatusMessage);
      new IHMCROS2Callback<>(ros2Node, getTopic(PlanOffsetStatus.class, robotName), this::acceptPlanOffsetStatus);
      new IHMCROS2Callback<>(ros2Node, getTopic(ControllerCrashNotificationPacket.class, robotName), this::acceptControllerCrashNotificationPacket);
      new IHMCROS2Callback<>(ros2Node, getTopic(CapturabilityBasedStatus.class, robotName), this::acceptCapturabilityBasedStatus);
      new IHMCROS2Callback<>(ros2Node, getTopic(WalkingStatusMessage.class, robotName), this::acceptWalkingStatusMessage);
   }

   public void registerAbortedListener(Notification abortedListener)
   {
      abortedListeners.add(abortedListener);
   }

   // TODO: Make a "snapshot" or "view" that would hold perspective/thread sensitive data?
   public void reset()
   {
      footstepTracker.reset();
      isWalking = false;
      latestKnownState = null;
      lastPlanOffset.setToZero();
      finishedWalkingNotification.poll();
      capturabilityBasedStatusTimer.reset();
      robotConfigurationDataTimer.reset();
   }

   private void acceptRobotConfigurationData(RobotConfigurationData message)
   {
      robotConfigurationDataTimer.reset();
      isWalkingFromConfigurationData = RobotMotionStatus.fromByte(message.getRobotMotionStatus()) == RobotMotionStatus.IN_MOTION;
   }

   private void acceptHighLevelStateChangeStatusMessage(HighLevelStateChangeStatusMessage message)
   {
      HighLevelControllerName initialState = HighLevelControllerName.fromByte(message.getInitialHighLevelControllerName());
      HighLevelControllerName endState = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());
      if (latestKnownState != initialState)
      {
         statusLogger.warn("We didn't know the state of the controller: ours: {} != controller said: {}", latestKnownState, initialState);
      }
      if (initialState == HighLevelControllerName.WALKING && endState != HighLevelControllerName.WALKING)
      {
         triggerNotWalkingStateAnymoreCallbacks();
      }
      statusLogger.info("Controller state changed from {} to {}", initialState, endState);
      latestKnownState = endState;
      footstepTracker.reset();
   }

   private void acceptWalkingControllerFailureStatusMessage(WalkingControllerFailureStatusMessage message)
   {
      triggerNotWalkingStateAnymoreCallbacks();
      statusLogger.error("Robot is falling! direction: {}", message.getFallingDirection());
      footstepTracker.reset();
   }

   private void acceptPlanOffsetStatus(PlanOffsetStatus message)
   {
      if (!message.getOffsetVector().epsilonEquals(lastPlanOffset, 1e-3))
      {
         statusLogger.info("Remaining footsteps shifted! offset vector: {}", message.getOffsetVector());
      }
      lastPlanOffset.set(message.getOffsetVector());
   }

   private void acceptControllerCrashNotificationPacket(ControllerCrashNotificationPacket message)
   {
      statusLogger.error("Controller crashed! {}", message::toString);
      triggerNotWalkingStateAnymoreCallbacks();
      footstepTracker.reset();
   }

   private void acceptCapturabilityBasedStatus(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      this.latestCapturabilityBasedStatus = capturabilityBasedStatus;
      capturabilityBasedStatusTimer.reset();
   }

   private void acceptWalkingStatusMessage(WalkingStatusMessage message)
   {
      // Declared locally since this represents the absolute state which other threads can access
      boolean isWalking = false;
      WalkingStatus walkingStatus = WalkingStatus.fromByte(message.getWalkingStatus());

      if (walkingStatus == WalkingStatus.STARTED || walkingStatus == WalkingStatus.RESUMED)
      {
         isWalking = true;

         statusLogger.info("Walking {}", walkingStatus == WalkingStatus.STARTED ? "started." : "resumed.");
      }
      else if (walkingStatus == WalkingStatus.ABORT_REQUESTED)
      {
         for (Notification abortedListener : abortedListeners)
         {
            abortedListener.set();
         }

         footstepTracker.reset();
         statusLogger.info("Walking aborted.");
      }
      else if (walkingStatus == WalkingStatus.PAUSED)
      {
         statusLogger.info("Walking paused.");
      }
      else
      {
         statusLogger.info("Walking completed.");
         footstepTracker.reset();
         finishedWalkingNotification.set();
      }

      this.isWalking = isWalking;
   }

   public boolean isWalking()
   {
      if (!robotConfigurationDataTimer.isExpired(CAPTURABILITY_BASED_STATUS_EXPIRATION_TIME))
         return isWalkingFromConfigurationData;

      return isWalking;
   }

   public Notification getFinishedWalkingNotification()
   {
      return finishedWalkingNotification;
   }

   public boolean isInWalkingState()
   {
      return capturabilityBasedStatusTimer.isRunning(CAPTURABILITY_BASED_STATUS_EXPIRATION_TIME);
   }

   private void triggerNotWalkingStateAnymoreCallbacks()
   {
      if (notWalkingStateAnymoreCallbackThrottler.run(2.0)) // don't call them more than once every few seconds
      {
         statusLogger.info("Calling \"not walking state anymore\" callbacks");
         for (Runnable notWalkingStateAnymoreCallback : notWalkingStateAnymoreCallbacks)
         {
            notWalkingStateAnymoreCallback.run();
         }
      }
   }
   
   public void addNotWalkingStateAnymoreCallback(Runnable callback)
   {
      notWalkingStateAnymoreCallbacks.add(callback);
   }

   public WalkingFootstepTracker getFootstepTracker()
   {
      return footstepTracker;
   }

   public void checkControllerIsRunning()
   {
      boolean controllerIsRunning = robotConfigurationDataTimer.isRunning(CAPTURABILITY_BASED_STATUS_EXPIRATION_TIME);

      if (!controllerIsRunning)
      {
         reset();
      }
   }

   public CapturabilityBasedStatus getLatestCapturabilityBasedStatus()
   {
      return latestCapturabilityBasedStatus;
   }
}

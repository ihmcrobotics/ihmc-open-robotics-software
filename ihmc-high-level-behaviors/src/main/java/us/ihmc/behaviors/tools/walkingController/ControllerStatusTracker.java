package us.ihmc.behaviors.tools.walkingController;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.log.LogTools;
import us.ihmc.tools.Timer;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.thread.Throttler;

import java.util.ArrayList;

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
   private final StatusLogger statusLogger;
   private volatile HighLevelControllerName latestKnownState;
   private final Vector3D lastPlanOffset = new Vector3D();
   private final Timer capturabilityBasedStatusTimer = new Timer();
   private volatile boolean isWalking = false;
   private final Notification finishedWalkingNotification = new Notification();
   private final ArrayList<Runnable> notWalkingStateAnymoreCallbacks = new ArrayList<>();
   private Throttler notWalkingStateAnymoreCallbackThrottler = new Throttler();

   public ControllerStatusTracker(StatusLogger statusLogger, ROS2NodeInterface ros2Node, String robotName)
   {
      this.statusLogger = statusLogger;
      footstepTracker = new WalkingFootstepTracker(ros2Node, robotName);

      finishedWalkingNotification.set();

      new IHMCROS2Callback<>(ros2Node, getTopic(HighLevelStateChangeStatusMessage.class, robotName), this::acceptHighLevelStateChangeStatusMessage);
      new IHMCROS2Callback<>(ros2Node, getTopic(WalkingControllerFailureStatusMessage.class, robotName), this::acceptWalkingControllerFailureStatusMessage);
      new IHMCROS2Callback<>(ros2Node, getTopic(PlanOffsetStatus.class, robotName), this::acceptPlanOffsetStatus);
      new IHMCROS2Callback<>(ros2Node, getTopic(ControllerCrashNotificationPacket.class, robotName), this::acceptControllerCrashNotificationPacket);
      new IHMCROS2Callback<>(ros2Node, getTopic(CapturabilityBasedStatus.class, robotName), this::acceptCapturabilityBasedStatus);
      new IHMCROS2Callback<>(ros2Node, getTopic(WalkingStatusMessage.class, robotName), this::acceptWalkingStatusMessage);
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
   }

   private void acceptHighLevelStateChangeStatusMessage(HighLevelStateChangeStatusMessage message)
   {
      HighLevelControllerName initialState = HighLevelControllerName.fromByte(message.getInitialHighLevelControllerName());
      HighLevelControllerName endState = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());
      if (latestKnownState != initialState)
      {
         LogTools.warn("We didn't know the state of the controller: ours: {} != controller said: {}", latestKnownState, initialState);
      }
      if (initialState == HighLevelControllerName.WALKING && endState != HighLevelControllerName.WALKING)
      {
         triggerNotWalkingStateAnymoreCallbacks();
      }
      statusLogger.info("Controller state changed from {} to {}", initialState, endState);
      latestKnownState = endState;
   }

   private void acceptWalkingControllerFailureStatusMessage(WalkingControllerFailureStatusMessage message)
   {
      triggerNotWalkingStateAnymoreCallbacks();
      statusLogger.error("Robot is falling! direction: {}", message.getFallingDirection());
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
   }

   private void acceptCapturabilityBasedStatus(CapturabilityBasedStatus message)
   {
      capturabilityBasedStatusTimer.reset();
   }

   private void acceptWalkingStatusMessage(WalkingStatusMessage message)
   {
      WalkingStatus walkingStatus = WalkingStatus.fromByte(message.getWalkingStatus());
      if (walkingStatus == WalkingStatus.STARTED || walkingStatus == WalkingStatus.RESUMED)
      {
         isWalking = true;
      }
      else if (walkingStatus == WalkingStatus.ABORT_REQUESTED)
      {

      }
      else
      {
         isWalking = false;
         finishedWalkingNotification.set();
      }
   }

   public boolean isWalking()
   {
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
}

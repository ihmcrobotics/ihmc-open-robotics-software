package us.ihmc.behaviors.tools.walkingController;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.tools.Timer;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.ros2.ROS2NodeInterface;

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
   private volatile HighLevelControllerName currentState;
   private final Vector3D lastPlanOffset = new Vector3D();
   private final Timer capturabilityBasedStatusTimer = new Timer();
   private volatile boolean isWalking = false;
   private final Notification finishedWalkingNotification = new Notification();

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

   public void reset()
   {
      footstepTracker.reset();
      isWalking = false;
      currentState = null;
      lastPlanOffset.setToZero();
      finishedWalkingNotification.poll();
      capturabilityBasedStatusTimer.reset();
   }

   private void acceptHighLevelStateChangeStatusMessage(HighLevelStateChangeStatusMessage message)
   {
      currentState = HighLevelControllerName.fromByte(message.getInitialHighLevelControllerName());
      statusLogger.info("Controller current state: {}", currentState);
   }

   private void acceptWalkingControllerFailureStatusMessage(WalkingControllerFailureStatusMessage message)
   {
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
      statusLogger.error("Controller crashed! {}", () -> message.toString());
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

   public WalkingFootstepTracker getFootstepTracker()
   {
      return footstepTracker;
   }
}

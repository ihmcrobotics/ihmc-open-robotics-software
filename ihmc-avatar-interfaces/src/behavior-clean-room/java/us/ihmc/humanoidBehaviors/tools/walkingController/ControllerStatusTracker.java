package us.ihmc.humanoidBehaviors.tools.walkingController;

import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.util.Timer;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.ros2.Ros2NodeInterface;

/**
 * A class to keep track of the controller by listening to its ROS 2 status API.
 *
 * WalkingControllerFailureStatusMessage - This is reported when robot is falling over
 * TextToSpeechPacket - All it does is say "walking" on walking start
 */
public class ControllerStatusTracker
{
   private static final double CAPTURABILITY_BASED_STATUS_EXPIRATION_TIME = 0.25;

   private final WalkingFootstepTracker footstepTracker;
   private volatile HighLevelControllerName currentState;
   private final Vector3D lastPlanOffset = new Vector3D();
   private final Timer capturabilityBasedStatusTimer = new Timer();
   private volatile boolean isWalking = false;
   private final Notification finishedWalkingNotification = new Notification();

   public ControllerStatusTracker(StatusLogger statusLogger, Ros2NodeInterface ros2Node, String robotName)
   {
      footstepTracker = new WalkingFootstepTracker(ros2Node, robotName);

      finishedWalkingNotification.set();

      new IHMCROS2Callback<>(ros2Node,
                             ControllerAPIDefinition.getTopic(HighLevelStateChangeStatusMessage.class, robotName),
                             message ->
                             {
                                currentState = HighLevelControllerName.fromByte(message.getInitialHighLevelControllerName());
                                statusLogger.info("Controller current state: {}", currentState);
                             });
      new IHMCROS2Callback<>(ros2Node,
                             ControllerAPIDefinition.getTopic(WalkingControllerFailureStatusMessage.class, robotName),
                             message ->
                             {
                                statusLogger.error("Robot is falling! direction: {}", message.getFallingDirection());
                             });
      new IHMCROS2Callback<>(ros2Node,
                             ControllerAPIDefinition.getTopic(PlanOffsetStatus.class, robotName),
                             message ->
                             {
                                if (!message.getOffsetVector().epsilonEquals(lastPlanOffset, 1e-3))
                                {
                                   statusLogger.info("Remaining footsteps shifted! offset vector: {}", message.getOffsetVector());
                                }
                                lastPlanOffset.set(message.getOffsetVector());
                             });
      new IHMCROS2Callback<>(ros2Node,
                             ControllerAPIDefinition.getTopic(ControllerCrashNotificationPacket.class, robotName),
                             message ->
                             {
                                statusLogger.error("Controller crashed! {}", () -> message.toString());
                             });
      new IHMCROS2Callback<>(ros2Node,
                             ControllerAPIDefinition.getTopic(CapturabilityBasedStatus.class, robotName),
                             message ->
                             {
                                capturabilityBasedStatusTimer.reset();
                             });
      new IHMCROS2Callback<>(ros2Node,
                             ControllerAPIDefinition.getTopic(WalkingStatusMessage.class, robotName),
                             message ->
                             {
                                WalkingStatus walkingStatus = WalkingStatus.fromByte(message.getWalkingStatus());
                                if (walkingStatus == WalkingStatus.STARTED  || walkingStatus == WalkingStatus.RESUMED)
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
                             });
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

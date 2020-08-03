package us.ihmc.humanoidBehaviors.tools.walkingController;

import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.util.Timer;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.ros2.Ros2NodeInterface;
import us.ihmc.tools.thread.PausablePeriodicThread;

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
   private volatile CapturabilityBasedStatus latestCapturabilityBasedStatus;
   private volatile long numberCapturabilityBasedStatusReceived;
   private final Vector3D lastPlanOffset = new Vector3D();
   private final Timer capturabilityBasedStatusTimer = new Timer();

   public ControllerStatusTracker(StatusLogger statusLogger, Ros2NodeInterface ros2Node, String robotName)
   {
      footstepTracker = new WalkingFootstepTracker(ros2Node, robotName);

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
//                                ++numberCapturabilityBasedStatusReceived;
//                                latestCapturabilityBasedStatus = message;
                             });
//      new IHMCROS2Callback<>(ros2Node,
//                             ControllerAPIDefinition.getTopic(WalkingStatusMessage.class, robotName),
//                             message ->
//                             {
//                                statusLogger.error("Controller crashed! {}", () -> message.toString());
//                             });

//      new PausablePeriodicThread("CapturabilityBSReporter", 1.0, () ->
//      {
//                                statusLogger.info("Capturability: # {}: {}",
//                                                   numberCapturabilityBasedStatusReceived,
//                                                   latestCapturabilityBasedStatus.toString());
//      }).start();
   }

   public boolean isInWalkingState()
   {
      return capturabilityBasedStatusTimer.isRunning(CAPTURABILITY_BASED_STATUS_EXPIRATION_TIME);
//      return currentState == HighLevelControllerName.WALKING;
   }

   public WalkingFootstepTracker getFootstepTracker()
   {
      return footstepTracker;
   }
}

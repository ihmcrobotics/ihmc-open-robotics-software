package us.ihmc.avatar.networkProcessor.reaStateUpdater;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.ros2.RealtimeRos2Node;

public class HumanoidAvatarREAStateUpdater
{
   private final RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.FAST_RTPS, "avatar_rea_state_updater");
   private final IHMCRealtimeROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final REAStateRequestMessage clearRequestMessage = new REAStateRequestMessage();
   private final REAStateRequestMessage pauseRequestMessage = new REAStateRequestMessage();
   private final REAStateRequestMessage resumeRequestMessage = new REAStateRequestMessage();
   private final REAStateRequestMessage clearAndResumeRequestMessage = new REAStateRequestMessage();

   public HumanoidAvatarREAStateUpdater(DRCRobotModel robotModel)
   {
      String robotName = robotModel.getSimpleRobotName();

      clearRequestMessage.setRequestClear(true);
      pauseRequestMessage.setRequestPause(true);
      resumeRequestMessage.setRequestResume(true);
      clearAndResumeRequestMessage.setRequestClear(true);
      clearAndResumeRequestMessage.setRequestResume(true);

      reaStateRequestPublisher = ROS2Tools.createPublisher(ros2Node, REAStateRequestMessage.class, REACommunicationProperties.subscriberTopicNameGenerator);
      ROS2Tools.createCallbackSubscription(ros2Node, HighLevelStateChangeStatusMessage.class, ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           this::handleHighLevelStateChangeMessage);
      ROS2Tools.createCallbackSubscription(ros2Node, WalkingStatusMessage.class, ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           this::handleWalkingStatusMessage);

      Runtime.getRuntime().addShutdownHook(new Thread(() -> {
         shutdown();
      }, "HumanoidAvatarREAStateUpdater-StopAll"));
   }

   private void handleHighLevelStateChangeMessage(Subscriber<HighLevelStateChangeStatusMessage> subscriber)
   {
      if (executorService.isShutdown())
         return;

      HighLevelStateChangeStatusMessage newMessage = subscriber.takeNextData();

      if (newMessage.getInitialHighLevelControllerName() == newMessage.getEndHighLevelControllerName())
         return;

      switch (HighLevelControllerName.fromByte(newMessage.getEndHighLevelControllerName()))
      {
      case WALKING:
         executorService.execute(() -> reaStateRequestPublisher.publish(clearAndResumeRequestMessage));
         break;
      default:
         executorService.execute(() -> reaStateRequestPublisher.publish(pauseRequestMessage));
         break;
      }
   }

   private void handleWalkingStatusMessage(Subscriber<WalkingStatusMessage> subscriber)
   {
      if (executorService.isShutdown())
         return;

      WalkingStatusMessage newMessage = subscriber.takeNextData();

      switch (WalkingStatus.fromByte(newMessage.getWalkingStatus()))
      {
      case STARTED:
         executorService.execute(() -> reaStateRequestPublisher.publish(pauseRequestMessage));
         break;
      case COMPLETED:
         executorService.execute(() -> reaStateRequestPublisher.publish(resumeRequestMessage));
         break;
      case ABORT_REQUESTED:
      default:
         // Do nothing?
         break;
      }
   }

   private void shutdown()
   {
      ros2Node.destroy();
      executorService.shutdownNow();
   }
}

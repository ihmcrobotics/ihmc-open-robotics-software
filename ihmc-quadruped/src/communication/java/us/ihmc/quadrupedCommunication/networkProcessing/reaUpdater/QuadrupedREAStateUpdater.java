package us.ihmc.quadrupedCommunication.networkProcessing.reaUpdater;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import quadruped_msgs.msg.dds.QuadrupedSteppingStateChangeMessage;
import perception_msgs.msg.dds.REAStateRequestMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.QuadrupedAPI;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.ros2.RealtimeROS2Node;

public class QuadrupedREAStateUpdater
{
   private final RealtimeROS2Node ros2Node;
   private final ROS2PublisherBasics<REAStateRequestMessage> reaStateRequestPublisher;

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final REAStateRequestMessage clearRequestMessage = new REAStateRequestMessage();
   private final REAStateRequestMessage pauseRequestMessage = new REAStateRequestMessage();
   private final REAStateRequestMessage resumeRequestMessage = new REAStateRequestMessage();
   private final REAStateRequestMessage clearAndResumeRequestMessage = new REAStateRequestMessage();

   public QuadrupedREAStateUpdater(String robotName, PubSubImplementation implementation)
   {
      clearRequestMessage.setRequestClear(true);
      pauseRequestMessage.setRequestPause(true);
      resumeRequestMessage.setRequestResume(true);
      clearAndResumeRequestMessage.setRequestClear(true);
      clearAndResumeRequestMessage.setRequestResume(true);

      ros2Node = ROS2Tools.createRealtimeROS2Node(implementation, "avatar_rea_state_updater");

      reaStateRequestPublisher = ros2Node.createPublisher(REACommunicationProperties.inputTopic.withTypeName(REAStateRequestMessage.class));
      ros2Node.createSubscription(QuadrupedAPI.getQuadrupedControllerOutputTopic(robotName)
                                              .withTypeName(HighLevelStateChangeStatusMessage.class), this::handleHighLevelStateChangeMessage);
      ros2Node.createSubscription(QuadrupedAPI.getQuadrupedControllerOutputTopic(robotName)
                                              .withTypeName(QuadrupedSteppingStateChangeMessage.class), this::handleSteppingStateMessage);

      Runtime.getRuntime().addShutdownHook(new Thread(() -> {
         shutdown();
      }, "HumanoidAvatarREAStateUpdater-StopAll"));

      ros2Node.spin();
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

   private void handleSteppingStateMessage(Subscriber<QuadrupedSteppingStateChangeMessage> subscriber)
   {
      if (executorService.isShutdown())
         return;

      QuadrupedSteppingStateChangeMessage newMessage = subscriber.takeNextData();

      switch (QuadrupedSteppingStateEnum.fromByte(newMessage.getEndQuadrupedSteppingStateEnum()))
      {
      case SOLE_WAYPOINT:
      case STEP:
         executorService.execute(() -> reaStateRequestPublisher.publish(pauseRequestMessage));
         break;
      case STAND:
         executorService.execute(() -> reaStateRequestPublisher.publish(resumeRequestMessage));
         break;
      default:
         // Do nothing?
         break;
      }
   }

   private void shutdown()
   {
      executorService.shutdownNow();
      ros2Node.destroy();
   }
}

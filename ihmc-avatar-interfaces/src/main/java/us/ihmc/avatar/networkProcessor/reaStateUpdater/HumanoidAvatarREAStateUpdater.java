package us.ihmc.avatar.networkProcessor.reaStateUpdater;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import perception_msgs.msg.dds.REAStateRequestMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.CloseableAndDisposable;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class HumanoidAvatarREAStateUpdater implements CloseableAndDisposable
{
   private final boolean manageROS2Node;
   private final RealtimeROS2Node ros2Node;
   private final ROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));

   private final REAStateRequestMessage clearRequestMessage = new REAStateRequestMessage();
   private final REAStateRequestMessage pauseRequestMessage = new REAStateRequestMessage();
   private final REAStateRequestMessage resumeRequestMessage = new REAStateRequestMessage();
   private final REAStateRequestMessage clearAndResumeRequestMessage = new REAStateRequestMessage();

   public HumanoidAvatarREAStateUpdater(DRCRobotModel robotModel, RealtimeROS2Node realtimeROS2Node)
   {
      this(robotModel, realtimeROS2Node, REACommunicationProperties.inputTopic);
   }

   public HumanoidAvatarREAStateUpdater(DRCRobotModel robotModel)
   {
      this(robotModel, REACommunicationProperties.inputTopic);
   }

   public HumanoidAvatarREAStateUpdater(DRCRobotModel robotModel, ROS2Topic<?> inputTopic)
   {
      this(robotModel, null, inputTopic);
   }

   private HumanoidAvatarREAStateUpdater(DRCRobotModel robotModel, RealtimeROS2Node realtimeROS2Node,
                                         ROS2Topic<?> inputTopic)
   {
      String robotName = robotModel.getSimpleRobotName();

      clearRequestMessage.setRequestClear(true);
      pauseRequestMessage.setRequestPause(true);
      resumeRequestMessage.setRequestResume(true);
      clearAndResumeRequestMessage.setRequestClear(true);
      clearAndResumeRequestMessage.setRequestResume(true);

      manageROS2Node = realtimeROS2Node == null;
      if (realtimeROS2Node == null)
         realtimeROS2Node = new ROS2NodeBuilder().buildRealtime("avatar_rea_state_updater");
      ros2Node = realtimeROS2Node;

      reaStateRequestPublisher = ros2Node.createPublisher(inputTopic.withTypeName(REAStateRequestMessage.class));
      ros2Node.createSubscription(HumanoidControllerAPI.getOutputTopic(robotName).withTypeName(HighLevelStateChangeStatusMessage.class), this::handleHighLevelStateChangeMessage);
      ros2Node.createSubscription(HumanoidControllerAPI.getOutputTopic(robotName).withTypeName(WalkingStatusMessage.class), this::handleWalkingStatusMessage);

      if (manageROS2Node)
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

   private void handleWalkingStatusMessage(Subscriber<WalkingStatusMessage> subscriber)
   {
      if (executorService.isShutdown())
         return;

      WalkingStatusMessage newMessage = subscriber.takeNextData();

      switch (WalkingStatus.fromByte(newMessage.getWalkingStatus()))
      {
         case STARTED:
         case RESUMED:
            executorService.execute(() -> reaStateRequestPublisher.publish(pauseRequestMessage));
            break;
         case COMPLETED:
         case PAUSED:
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
      executorService.shutdownNow();
      if (manageROS2Node)
         ros2Node.destroy();
   }

   @Override
   public void closeAndDispose()
   {
      shutdown();
   }
}

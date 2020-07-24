package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.REASensorDataFilterParametersMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.ros.REAModuleROS2Subscription;
import us.ihmc.robotEnvironmentAwareness.ros.REASourceType;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.RegionFeaturesProvider;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2Node;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.inputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberCustomRegionsTopicName;

public class RealSenseREANetworkProvider implements REANetworkProvider
{
   private final IHMCROS2Publisher<PlanarRegionsListMessage> stereoRegionPublisher;

   private final Ros2Node ros2Node;

   private PlanarRegionsListMessage lastPlanarRegionsListMessage;

   public RealSenseREANetworkProvider(ROS2Topic stereoOutputTopic)
   {
      this(ROS2Tools.createRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME), stereoOutputTopic);
   }

   public RealSenseREANetworkProvider(Ros2Node ros2Node, ROS2Topic stereoOutputTopic)
   {
      this.ros2Node = ros2Node;

      stereoRegionPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PlanarRegionsListMessage.class, stereoOutputTopic);
   }

   @Override
   public void registerMessager(Messager messager)
   {
   }

   @Override
   public void update(RegionFeaturesProvider regionFeaturesProvider, boolean planarRegionsHaveBeenUpdated)
   {
      if (regionFeaturesProvider.getPlanarRegionsList() == null)
         return;

      if (regionFeaturesProvider.getPlanarRegionsList().isEmpty())
         return;

      if (planarRegionsHaveBeenUpdated)
         lastPlanarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionFeaturesProvider.getPlanarRegionsList());

      stereoRegionPublisher.publish(lastPlanarRegionsListMessage);
   }

   @Override
   public void publishCurrentState()
   {
   }

   @Override
   public void registerStereoVisionPointCloudHandler(NewMessageListener<StereoVisionPointCloudMessage> stereoVisionPointCloudHandler)
   {
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           StereoVisionPointCloudMessage.class,
                                           REASourceType.STEREO_POINT_CLOUD.getTopicName(),
                                           stereoVisionPointCloudHandler);
   }

   @Override
   public void registerStereoVisionPointCloudHandler(Messager messager, NewMessageListener<StereoVisionPointCloudMessage> stereoVisionPointCloudHandler)
   {
      new REAModuleROS2Subscription<>(ros2Node, messager, REASourceType.STEREO_POINT_CLOUD, StereoVisionPointCloudMessage.class, stereoVisionPointCloudHandler);
   }

   @Override
   public void registerCustomRegionsHandler(NewMessageListener<PlanarRegionsListMessage> customRegionsHandler)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, subscriberCustomRegionsTopicName, customRegionsHandler);
   }

   @Override
   public void registerREAStateRequestHandler(NewMessageListener<REAStateRequestMessage> requestHandler)
   {
   }

   @Override
   public void registerREASensorDataFilterParametersHandler(NewMessageListener<REASensorDataFilterParametersMessage> parametersHandler)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, REASensorDataFilterParametersMessage.class, inputTopic, parametersHandler);
   }

   @Override
   public void stop()
   {
      ros2Node.destroy();
   }
}

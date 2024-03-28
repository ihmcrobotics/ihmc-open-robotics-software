package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import perception_msgs.msg.dds.*;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.converters.REAParametersMessageHelper;
import us.ihmc.robotEnvironmentAwareness.ros.REAModuleROS2Subscription;
import us.ihmc.robotEnvironmentAwareness.ros.REASourceType;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.RegionFeaturesProvider;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberCustomRegionsTopicName;

public class RealSenseREANetworkProvider implements REANetworkProvider
{
   private final ROS2PublisherBasics<PlanarRegionsListMessage> stereoRegionPublisher;
   private final ROS2PublisherBasics<OcTreeKeyListMessage> ocTreePublisher;

   private final ROS2Node ros2Node;
   private final ROS2Topic inputTopic;

   private PlanarRegionsListMessage lastPlanarRegionsListMessage;

   public RealSenseREANetworkProvider(ROS2Topic inputTopic, ROS2Topic stereoOutputTopic)
   {
      this(ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, PerceptionAPI.REA_NODE_NAME), inputTopic, stereoOutputTopic);
   }

   public RealSenseREANetworkProvider(ROS2Node ros2Node, ROS2Topic inputTopic, ROS2Topic stereoOutputTopic)
   {
      this.ros2Node = ros2Node;
      this.inputTopic = inputTopic;

      stereoRegionPublisher = ros2Node.createPublisher(stereoOutputTopic.withTypeName(PlanarRegionsListMessage.class));
      ocTreePublisher = ros2Node.createPublisher(stereoOutputTopic.withTypeName(OcTreeKeyListMessage.class));
   }

   @Override
   public void registerMessager(Messager messager)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    NormalEstimationParametersMessage.class,
                                                    inputTopic,
                                                    s -> messager.submitMessage(REAModuleAPI.NormalEstimationParameters,
                                                                                REAParametersMessageHelper.convertFromMessage(s.takeNextData())));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    PlanarRegionSegmentationParametersMessage.class,
                                                    inputTopic,
                                                    s -> messager.submitMessage(REAModuleAPI.PlanarRegionsSegmentationParameters,
                                                                                REAParametersMessageHelper.convertFromMessage(s.takeNextData())));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    PolygonizerParametersMessage.class,
                                                    inputTopic,
                                                    s -> messager.submitMessage(REAModuleAPI.PlanarRegionsPolygonizerParameters, s.takeNextData()));
   }

   @Override
   public void update(RegionFeaturesProvider regionFeaturesProvider, boolean planarRegionsHaveBeenUpdated, NormalOcTree ocTree)
   {
      if (regionFeaturesProvider.getPlanarRegionsList() != null && !regionFeaturesProvider.getPlanarRegionsList().isEmpty())
      {
         if (planarRegionsHaveBeenUpdated)
            lastPlanarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionFeaturesProvider.getPlanarRegionsList());

         stereoRegionPublisher.publish(lastPlanarRegionsListMessage);
      }

      if (ocTree != null && ocTree.getRoot() != null)
      {
         OcTreeKeyListMessage ocTreeMessage = OcTreeMessageConverter.createOcTreeDataMessage(ocTree);
         ocTreePublisher.publish(ocTreeMessage);
      }
   }

   @Override
   public void publishCurrentState()
   {
   }

   @Override
   public void registerStereoVisionPointCloudHandler(NewMessageListener<StereoVisionPointCloudMessage> stereoVisionPointCloudHandler)
   {
      ros2Node.createSubscription(PerceptionAPI.D435_POINT_CLOUD, stereoVisionPointCloudHandler);
   }

   @Override
   public void registerStereoVisionPointCloudHandler(Messager messager, NewMessageListener<StereoVisionPointCloudMessage> stereoVisionPointCloudHandler)
   {
      new REAModuleROS2Subscription<>(ros2Node,
                                      messager,
                                      REASourceType.STEREO_POINT_CLOUD,
                                      StereoVisionPointCloudMessage.class,
                                      stereoVisionPointCloudHandler,
                                      ROS2QosProfile.BEST_EFFORT());
   }

   @Override
   public void registerCustomRegionsHandler(NewMessageListener<PlanarRegionsListMessage> customRegionsHandler)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, subscriberCustomRegionsTopicName, customRegionsHandler);
   }

   @Override
   public void registerREAStateRequestHandler(NewMessageListener<REAStateRequestMessage> requestHandler)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, REAStateRequestMessage.class, inputTopic, requestHandler);
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

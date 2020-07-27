package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.ros.REAModuleROS2Subscription;
import us.ihmc.robotEnvironmentAwareness.ros.REASourceType;
import us.ihmc.robotEnvironmentAwareness.updaters.REACurrentStateProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.RegionFeaturesProvider;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.inputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberCustomRegionsTopicName;

public class LidarREANetworkProvider implements REANetworkProvider
{
   private final IHMCROS2Publisher<PlanarRegionsListMessage> planarRegionPublisher;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> lidarRegionPublisher;

   private REACurrentStateProvider currentStateProvider = null;


   private final Ros2Node ros2Node;

   private final ROS2Topic<PlanarRegionsListMessage> outputTopic;
   private PlanarRegionsListMessage lastPlanarRegionsListMessage;

   public LidarREANetworkProvider(ROS2Topic outputTopic,
                                  ROS2Topic lidarOutputTopic)
   {
      this(ROS2Tools.createRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME),
           outputTopic,
           lidarOutputTopic);
   }

   public LidarREANetworkProvider(Ros2Node ros2Node,
                                  ROS2Topic outputTopic,
                                  ROS2Topic lidarOutputTopic)
   {
      this.ros2Node = ros2Node;
      this.outputTopic = outputTopic;

      planarRegionPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PlanarRegionsListMessage.class, outputTopic);
      lidarRegionPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PlanarRegionsListMessage.class, lidarOutputTopic);
   }

   public void registerMessager(Messager messager)
   {
      currentStateProvider = new REACurrentStateProvider(ros2Node, outputTopic, messager);
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

      planarRegionPublisher.publish(lastPlanarRegionsListMessage);
      lidarRegionPublisher.publish(lastPlanarRegionsListMessage);
   }

   @Override
   public void publishCurrentState()
   {
      if (currentStateProvider != null)
         currentStateProvider.publishCurrentState();
   }

   @Override
   public void registerLidarScanHandler(NewMessageListener<LidarScanMessage> lidarScanHandler)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, LidarScanMessage.class, REASourceType.LIDAR_SCAN.getTopicName(), lidarScanHandler);
   }


   @Override
   public void registerLidarScanHandler(Messager messager, NewMessageListener<LidarScanMessage> lidarScanHandler)
   {
      new REAModuleROS2Subscription<>(ros2Node, messager, REASourceType.LIDAR_SCAN, LidarScanMessage.class, lidarScanHandler);
   }

   @Override
   public void registerCustomRegionsHandler(NewMessageListener<PlanarRegionsListMessage> customRegionsHandler)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, subscriberCustomRegionsTopicName, customRegionsHandler);
   }

   @Override
   public void registerPlanarRegionsListRequestHandler(NewMessageListener<RequestPlanarRegionsListMessage> requestHandler)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RequestPlanarRegionsListMessage.class, inputTopic, requestHandler);
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

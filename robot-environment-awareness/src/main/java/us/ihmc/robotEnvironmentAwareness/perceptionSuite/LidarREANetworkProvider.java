package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import perception_msgs.msg.dds.LidarScanMessage;
import perception_msgs.msg.dds.NormalEstimationParametersMessage;
import perception_msgs.msg.dds.OcTreeKeyListMessage;
import perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.PolygonizerParametersMessage;
import perception_msgs.msg.dds.REASensorDataFilterParametersMessage;
import perception_msgs.msg.dds.REAStateRequestMessage;
import perception_msgs.msg.dds.RequestPlanarRegionsListMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.converters.REAParametersMessageHelper;
import us.ihmc.robotEnvironmentAwareness.ros.REAModuleROS2Subscription;
import us.ihmc.robotEnvironmentAwareness.ros.REASourceType;
import us.ihmc.robotEnvironmentAwareness.updaters.REACurrentStateProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.RegionFeaturesProvider;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.inputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberCustomRegionsTopicName;

public class LidarREANetworkProvider implements REANetworkProvider
{
   private final ROS2Publisher<PlanarRegionsListMessage> planarRegionPublisher;
   private final ROS2Publisher<PlanarRegionsListMessage> lidarRegionPublisher;
   private final ROS2Publisher<OcTreeKeyListMessage> ocTreePublisher;

   private REACurrentStateProvider currentStateProvider = null;

   private final ROS2Node ros2Node;

   private final ROS2Topic<PlanarRegionsListMessage> outputTopic;
   private PlanarRegionsListMessage lastPlanarRegionsListMessage;

   public LidarREANetworkProvider(ROS2Topic outputTopic, ROS2Topic lidarOutputTopic)
   {
      this(new ROS2NodeBuilder().build(PerceptionAPI.REA_NODE_NAME), outputTopic, lidarOutputTopic);
   }

   public LidarREANetworkProvider(ROS2Node ros2Node, ROS2Topic outputTopic, ROS2Topic lidarOutputTopic)
   {
      this.ros2Node = ros2Node;
      this.outputTopic = outputTopic;

      planarRegionPublisher = ros2Node.createPublisher(outputTopic.withTypeName(PlanarRegionsListMessage.class));
      lidarRegionPublisher = ros2Node.createPublisher(lidarOutputTopic.withTypeName(PlanarRegionsListMessage.class));
      ocTreePublisher = ros2Node.createPublisher(outputTopic.withTypeName(OcTreeKeyListMessage.class));
   }

   public void registerMessager(Messager messager)
   {
      currentStateProvider = new REACurrentStateProvider(ros2Node, outputTopic, messager);

      ros2Node.createSubscription(inputTopic.withTypeName(NormalEstimationParametersMessage.class),
                                  s -> messager.submitMessage(REAModuleAPI.NormalEstimationParameters, REAParametersMessageHelper.convertFromMessage(s.takeNextData())));
      ros2Node.createSubscription(inputTopic.withTypeName(PlanarRegionSegmentationParametersMessage.class),
                                  s -> messager.submitMessage(REAModuleAPI.PlanarRegionsSegmentationParameters,
                                                               REAParametersMessageHelper.convertFromMessage(s.takeNextData())));
      ros2Node.createSubscription(inputTopic.withTypeName(PolygonizerParametersMessage.class),
                                  s -> messager.submitMessage(REAModuleAPI.PlanarRegionsPolygonizerParameters, s.takeNextData()));
   }

   @Override
   public void update(RegionFeaturesProvider regionFeaturesProvider, boolean planarRegionsHaveBeenUpdated, NormalOcTree ocTree)
   {
      if (regionFeaturesProvider.getPlanarRegionsList() != null && !regionFeaturesProvider.getPlanarRegionsList().isEmpty())
      {
         if (planarRegionsHaveBeenUpdated)
            lastPlanarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionFeaturesProvider.getPlanarRegionsList());

         planarRegionPublisher.publish(lastPlanarRegionsListMessage);
         lidarRegionPublisher.publish(lastPlanarRegionsListMessage);
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
      if (currentStateProvider != null)
         currentStateProvider.publishCurrentState();
   }

   @Override
   public void registerLidarScanHandler(NewMessageListener<LidarScanMessage> lidarScanHandler)
   {
      ros2Node.createSubscription(LidarScanMessage.class, lidarScanHandler, REASourceType.LIDAR_SCAN.getTopicName());
   }

   @Override
   public void registerLidarScanHandler(Messager messager, NewMessageListener<LidarScanMessage> lidarScanHandler)
   {
      new REAModuleROS2Subscription<>(ros2Node, messager, REASourceType.LIDAR_SCAN, LidarScanMessage.class, lidarScanHandler, ROS2QosProfile.BEST_EFFORT());
   }

   @Override
   public void registerCustomRegionsHandler(NewMessageListener<PlanarRegionsListMessage> customRegionsHandler)
   {
      ros2Node.createSubscription(subscriberCustomRegionsTopicName.withTypeName(PlanarRegionsListMessage.class), customRegionsHandler);
   }

   @Override
   public void registerPlanarRegionsListRequestHandler(NewMessageListener<RequestPlanarRegionsListMessage> requestHandler)
   {
      ros2Node.createSubscription(inputTopic.withTypeName(RequestPlanarRegionsListMessage.class), requestHandler);
   }

   @Override
   public void registerREAStateRequestHandler(NewMessageListener<REAStateRequestMessage> requestHandler)
   {
      ros2Node.createSubscription(inputTopic.withTypeName(REAStateRequestMessage.class), requestHandler);
   }

   @Override
   public void registerREASensorDataFilterParametersHandler(NewMessageListener<REASensorDataFilterParametersMessage> parametersHandler)
   {
      ros2Node.createSubscription(inputTopic.withTypeName(REASensorDataFilterParametersMessage.class), parametersHandler);
   }

   @Override
   public void stop()
   {
      ros2Node.destroy();
   }
}

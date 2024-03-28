package us.ihmc.robotEnvironmentAwareness.updaters;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.inputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberCustomRegionsTopicName;

import java.util.concurrent.atomic.AtomicReference;

import perception_msgs.msg.dds.LidarScanMessage;
import perception_msgs.msg.dds.NormalEstimationParametersMessage;
import perception_msgs.msg.dds.OcTreeKeyListMessage;
import perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.PolygonizerParametersMessage;
import perception_msgs.msg.dds.REASensorDataFilterParametersMessage;
import perception_msgs.msg.dds.REAStateRequestMessage;
import perception_msgs.msg.dds.RequestPlanarRegionsListMessage;
import ihmc_common_msgs.msg.dds.StampedPosePacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
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
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

public class REAPlanarRegionPublicNetworkProvider implements REANetworkProvider
{
   private static final boolean publishOctree = false;

   private final ROS2PublisherBasics<PlanarRegionsListMessage> planarRegionPublisher;
   private final ROS2PublisherBasics<PlanarRegionsListMessage> lidarRegionPublisher;
   private final ROS2PublisherBasics<PlanarRegionsListMessage> stereoRegionPublisher;
   private final ROS2PublisherBasics<PlanarRegionsListMessage> depthRegionPublisher;
   private final ROS2PublisherBasics<OcTreeKeyListMessage> ocTreePublisher;

   private REACurrentStateProvider currentStateProvider = null;
   private AtomicReference<Boolean> isUsingLidar, isUsingStereoVision, isUsingDepthCloud;

   private final ROS2NodeInterface ros2Node;

   private final ROS2Topic<PlanarRegionsListMessage> outputTopic;
   private PlanarRegionsListMessage lastPlanarRegionsListMessage;

   public REAPlanarRegionPublicNetworkProvider(ROS2Topic outputTopic, ROS2Topic lidarOutputTopic, ROS2Topic stereoOutputTopic, ROS2Topic depthOutputTopic)
   {
      this(ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, PerceptionAPI.REA_NODE_NAME), outputTopic, lidarOutputTopic, stereoOutputTopic,
           depthOutputTopic);
   }

   public REAPlanarRegionPublicNetworkProvider(ROS2NodeInterface ros2Node,
                                               ROS2Topic outputTopic,
                                               ROS2Topic lidarOutputTopic,
                                               ROS2Topic stereoOutputTopic,
                                               ROS2Topic depthOutputTopic)
   {
      this.ros2Node = ros2Node;
      this.outputTopic = outputTopic;

      planarRegionPublisher = ros2Node.createPublisher(outputTopic.withTypeName(PlanarRegionsListMessage.class));
      lidarRegionPublisher = ros2Node.createPublisher(lidarOutputTopic.withTypeName(PlanarRegionsListMessage.class));
      stereoRegionPublisher = ros2Node.createPublisher(stereoOutputTopic.withTypeName(PlanarRegionsListMessage.class));
      depthRegionPublisher = ros2Node.createPublisher(depthOutputTopic.withTypeName(PlanarRegionsListMessage.class));
      ocTreePublisher = ros2Node.createPublisher(outputTopic.withTypeName(OcTreeKeyListMessage.class));
   }

   public void registerMessager(Messager messager)
   {
      currentStateProvider = new REACurrentStateProvider(ros2Node, outputTopic, messager);

      // This should be the only input with a default value, the rest gets populated at the very start.
      isUsingLidar = messager.createInput(REAModuleAPI.LidarBufferEnable);
      isUsingStereoVision = messager.createInput(REAModuleAPI.StereoVisionBufferEnable);
      isUsingDepthCloud = messager.createInput(REAModuleAPI.DepthCloudBufferEnable);

      ros2Node.createSubscription(inputTopic.withTypeName(NormalEstimationParametersMessage.class), s -> messager.submitMessage(REAModuleAPI.NormalEstimationParameters,
                                                                                                                                 REAParametersMessageHelper.convertFromMessage(
                                                                                                                                                        s.takeNextData())));
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
         if (isUsingLidar != null && isUsingLidar.get())
            lidarRegionPublisher.publish(lastPlanarRegionsListMessage);
         if (isUsingStereoVision != null && isUsingStereoVision.get())
            stereoRegionPublisher.publish(lastPlanarRegionsListMessage);
         if (isUsingDepthCloud != null && isUsingDepthCloud.get())
            depthRegionPublisher.publish(lastPlanarRegionsListMessage);
      }

      if (publishOctree && ocTree != null && ocTree.getRoot() != null)
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
      String topicName = REASourceType.LIDAR_SCAN.getTopicName();
      ros2Node.createSubscription(LidarScanMessage.class, lidarScanHandler, topicName, ROS2QosProfile.BEST_EFFORT());
   }

   @Override
   public void registerStereoVisionPointCloudHandler(NewMessageListener<StereoVisionPointCloudMessage> stereoVisionPointCloudHandler)
   {
      String topicName = REASourceType.STEREO_POINT_CLOUD.getTopicName();
      ros2Node.createSubscription(StereoVisionPointCloudMessage.class, stereoVisionPointCloudHandler, topicName, ROS2QosProfile.BEST_EFFORT());
   }

   @Override
   public void registerLidarScanHandler(Messager messager, NewMessageListener<LidarScanMessage> lidarScanHandler)
   {
      new REAModuleROS2Subscription<>(ros2Node, messager, REASourceType.LIDAR_SCAN, LidarScanMessage.class, lidarScanHandler, ROS2QosProfile.BEST_EFFORT());
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
   public void registerDepthPointCloudHandler(Messager messager, NewMessageListener<StereoVisionPointCloudMessage> depthPointCloudHandler)
   {
      new REAModuleROS2Subscription<>(ros2Node,
                                      messager,
                                      REASourceType.DEPTH_POINT_CLOUD,
                                      StereoVisionPointCloudMessage.class,
                                      depthPointCloudHandler,
                                      ROS2QosProfile.BEST_EFFORT());
   }

   @Override
   public void registerStampedPosePacketHandler(Messager messager, NewMessageListener<StampedPosePacket> stampedPosePacketHandler)
   {
      new REAModuleROS2Subscription<>(ros2Node,
                                      messager,
                                      "/ihmc/stamped_pose_T265",
                                      StampedPosePacket.class,
                                      stampedPosePacketHandler,
                                      ROS2QosProfile.BEST_EFFORT(),
                                      REAModuleAPI.DepthCloudBufferEnable);
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
      if (ros2Node.getName().equals(PerceptionAPI.REA_NODE_NAME))
         ((ROS2Node) ros2Node).destroy();
   }
}

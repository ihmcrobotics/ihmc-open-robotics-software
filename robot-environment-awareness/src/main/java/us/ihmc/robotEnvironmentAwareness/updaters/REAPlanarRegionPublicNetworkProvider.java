package us.ihmc.robotEnvironmentAwareness.updaters;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.ros2.Ros2Node;

public class REAPlanarRegionPublicNetworkProvider
{
   private final IHMCROS2Publisher<PlanarRegionsListMessage> publisher;

   private final RegionFeaturesProvider regionFeaturesProvider;

   public REAPlanarRegionPublicNetworkProvider(RegionFeaturesProvider regionFeaturesProvider, Ros2Node ros2Node,
                                               MessageTopicNameGenerator publisherTopicNameGenerator, MessageTopicNameGenerator subscriberTopicNameGenerator)
   {
      this.regionFeaturesProvider = regionFeaturesProvider;
      publisher = ROS2Tools.createPublisher(ros2Node, PlanarRegionsListMessage.class, publisherTopicNameGenerator);
   }

   private PlanarRegionsListMessage lastPlanarRegionsListMessage;

   public void update(boolean planarRegionsHaveBeenUpdated)
   {
      if (regionFeaturesProvider.getPlanarRegionsList() == null)
         return;

      if (regionFeaturesProvider.getPlanarRegionsList().isEmpty())
         return;

      if (planarRegionsHaveBeenUpdated)
         lastPlanarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionFeaturesProvider.getPlanarRegionsList());

      publisher.publish(lastPlanarRegionsListMessage);
   }
}

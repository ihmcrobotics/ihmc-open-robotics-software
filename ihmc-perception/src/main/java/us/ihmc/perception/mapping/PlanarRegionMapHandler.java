package us.ihmc.perception.mapping;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;

public class PlanarRegionMapHandler
{
   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;
   private PlanarRegionsList planarRegions;
   private PlanarRegionFilteredMap filteredMap;

   public PlanarRegionMapHandler()
   {
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "filtered_map_node");

      ros2Helper = new ROS2Helper(ros2Node);

      ros2Helper.subscribeViaCallback(ROS2Tools.MAPSENSE_REGIONS, this::planarRegionCallback);
   }

   public void planarRegionCallback(PlanarRegionsListMessage planarRegionsListMessage)
   {
      planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
      filteredMap.submitRegions(planarRegions);
   }

   public PlanarRegionsList getMapRegions()
   {
      return filteredMap.getMapRegions();
   }

   public PlanarRegionFilteredMap getFilteredMap()
   {
      return filteredMap;
   }
}

package us.ihmc.perception.steppableRegions;

import perception_msgs.msg.dds.SteppableRegionsListCollectionMessage;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

public class SteppableRegionsManager
{
   private final ROS2Publisher<SteppableRegionsListCollectionMessage> publisher;
   SteppableRegionsCalculationModule steppableRegionsCalculationModule;

   public SteppableRegionsManager(ROS2Node ros2Node)
   {
      steppableRegionsCalculationModule = new SteppableRegionsCalculationModule();

      publisher = ros2Node.createPublisher(SteppableRegionsAPI.STEPPABLE_REGIONS_OUTPUT);
   }

   public void update(TerrainMapData terrainMapData)
   {
      steppableRegionsCalculationModule.compute(terrainMapData);

      publishSteppableRegionsMessage();
   }

   public void publishSteppableRegionsMessage()
   {
      SteppableRegionsListCollection steppableRegionsListCollection = steppableRegionsCalculationModule.getSteppableRegionsListCollection();

      SteppableRegionsListCollectionMessage steppableRegionsListCollectionMessage = SteppableRegionMessageConverter.convertToSteppableRegionsListCollectionMessage(
            steppableRegionsListCollection);

      publisher.publish(steppableRegionsListCollectionMessage);
   }

   public void destroy()
   {
      publisher.remove();
   }
}

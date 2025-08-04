package us.ihmc.footstepPlanning;

import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.footstepPlanning.communication.ContinuousHikingAPI;
import us.ihmc.footstepPlanning.steppableRegions.TerrainMapData;
import us.ihmc.footstepPlanning.steppableRegions.TerrainMapTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;

public class SnappingTerrainManager
{
   private final SnappingTerrainExtractor snappingTerrainExtractor;
   private ROS2Publisher<TerrainMapMessage> snappingTerrainPublisher;

   public SnappingTerrainManager(HeightMapParameters heightMapParameters)
   {
      this(null, heightMapParameters);
   }

   public SnappingTerrainManager(ROS2Node ros2Node, HeightMapParameters heightMapParameters)
   {
      if (ros2Node != null)
         snappingTerrainPublisher = ros2Node.createPublisher(ContinuousHikingAPI.TERRAIN_MAP);

      snappingTerrainExtractor = new SnappingTerrainExtractor(heightMapParameters);
   }

   public void updateAndPublish(HeightMapData heightMapData)
   {
      snappingTerrainExtractor.update(heightMapData);

      publishTerrainMapData(snappingTerrainExtractor.getTerrainMapData());
   }

   private void publishTerrainMapData(TerrainMapData terrainMapData)
   {
      TerrainMapMessage message = TerrainMapTools.toMessage(terrainMapData);
      if (snappingTerrainPublisher != null)
         snappingTerrainPublisher.publish(message);
   }

   public TerrainMapData getTerrainMapData()
   {
      return snappingTerrainExtractor.getTerrainMapData();
   }

   public void close()
   {
      snappingTerrainExtractor.close();
   }
}

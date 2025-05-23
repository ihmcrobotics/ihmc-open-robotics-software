package us.ihmc.footstepPlanning;

import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.footstepPlanning.communication.ContinuousHikingAPI;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.heightMap.TerrainMapTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

public class SnappingTerrainManager
{
   private final SnappingTerrainExtractor snappingTerrainExtractor;
   private final ROS2Publisher<TerrainMapMessage> snappingTerrainPublisher;

   public SnappingTerrainManager(ROS2Node ros2Node, HeightMapParameters heightMapParameters)
   {
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

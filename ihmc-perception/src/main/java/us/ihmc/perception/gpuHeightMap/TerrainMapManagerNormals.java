package us.ihmc.perception.gpuHeightMap;

import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.perception.heightMap.SteppableRegionCalculatorParameters;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;

public class TerrainMapManagerNormals
{
   private final TerrainMapExtractor terrainMapExtractor;
   private ROS2Publisher<TerrainMapMessage> snappingTerrainPublisher;
   private final TerrainMapMessage terrainMapMessage;
   private long sequenceId = 0;

   public TerrainMapManagerNormals(ROS2Node ros2Node, HeightMapParameters heightMapParameters, SteppableRegionCalculatorParameters footstepPlannerParameters)
   {
      if (ros2Node != null)
         snappingTerrainPublisher = ros2Node.createPublisher(PerceptionAPI.TERRAIN_MAP);

      terrainMapMessage = new TerrainMapMessage();
      terrainMapExtractor = new TerrainMapExtractor(heightMapParameters, footstepPlannerParameters);
   }

   public void updateAndPublish(HeightMapData heightMapData)
   {
      terrainMapExtractor.update(heightMapData);
      publishTerrainMapData(terrainMapExtractor.getTerrainMapData());
   }

   private void publishTerrainMapData(TerrainMapData terrainMapData)
   {
      TerrainMapMessageTools.toMessage(terrainMapData, terrainMapMessage);
      terrainMapMessage.setSequenceId(sequenceId++);

      if (snappingTerrainPublisher != null)
         snappingTerrainPublisher.publish(terrainMapMessage);
   }

   public TerrainMapData getTerrainMapData()
   {
      return terrainMapExtractor.getTerrainMapData();
   }

   public void close()
   {
      terrainMapExtractor.destroy();
   }
}

package us.ihmc.behaviors.activeMapping;

import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.filters.DepthImageFilteringParameters;
import us.ihmc.perception.gpuMapping.HeightMapParameters;
import us.ihmc.perception.gpuMapping.TerrainMapParameters;

public class ActiveMappingParameterToolBox
{
   private final HeightMapParameters heightMapParameters;
   private final ROS2StoredPropertySetGroup ros2StoredPropertySetGroup;
   private final DepthImageFilteringParameters depthImageFilteringParameters;
   private final TerrainMapParameters terrainMapParameters;

   public ActiveMappingParameterToolBox(ROS2Node ros2Node)
   {
      ros2StoredPropertySetGroup = new ROS2StoredPropertySetGroup(ros2Node);

      heightMapParameters = new HeightMapParameters();
      depthImageFilteringParameters = new DepthImageFilteringParameters();
      terrainMapParameters = new TerrainMapParameters();

      // Add Parameters to be synced between the UI and this process
      ros2StoredPropertySetGroup.registerStoredPropertySet(PerceptionComms.HEIGHT_MAP_PARAMETERS, heightMapParameters);
      ros2StoredPropertySetGroup.registerStoredPropertySet(PerceptionComms.DEPTH_IMAGE_FILTERING_PARAMETERS, depthImageFilteringParameters);
      ros2StoredPropertySetGroup.registerStoredPropertySet(PerceptionComms.TERRAIN_MAP_PARAMETERS, terrainMapParameters);
   }

   public void update()
   {
      ros2StoredPropertySetGroup.update();
   }

   public HeightMapParameters getHeightMapParameters()
   {
      return heightMapParameters;
   }

   public DepthImageFilteringParameters getDepthImageFilteringParameters()
   {
      return depthImageFilteringParameters;
   }

   public TerrainMapParameters getTerrainMapParameters()
   {
      return terrainMapParameters;
   }
}

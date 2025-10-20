package us.ihmc.perception.gpuMapping;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ActiveMappingProcessParametersBasics extends ActiveMappingProcessParametersReadOnly, StoredPropertySetBasics
{
   default void setRunHeightMap(boolean runHeightMap)
   {
      set(ActiveMappingProcessParameters.runHeightMap, runHeightMap);
   }

   default void setRunTerrainMap(boolean runTerrainMap)
   {
      set(ActiveMappingProcessParameters.runTerrainMap, runTerrainMap);
   }

   default void setRunD457TunableTransform(boolean runD457TunableTransform)
   {
      set(ActiveMappingProcessParameters.runD457TunableTransform, runD457TunableTransform);
   }

   default void setPublishHeightMap(boolean publishHeightMap)
   {
      set(ActiveMappingProcessParameters.publishHeightMap, publishHeightMap);
   }

   default void setPublishTerrainMap(boolean publishTerrainMap)
   {
      set(ActiveMappingProcessParameters.publishTerrainMap, publishTerrainMap);
   }
}

package us.ihmc.perception.gpuMapping;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ActiveMappingProcessParametersBasics extends ActiveMappingProcessParametersReadOnly, StoredPropertySetBasics
{
   default void setRunOnboardKernels(boolean runOnboardKernels)
   {
      set(ActiveMappingProcessParameters.runOnboardKernels, runOnboardKernels);
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

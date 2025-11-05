package us.ihmc.perception.gpuMapping;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.gpuMapping.ActiveMappingProcessParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ActiveMappingProcessParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean getRunOnboardKernels()
   {
      return get(runOnboardKernels);
   }

   default boolean getPublishHeightMap()
   {
      return get(publishHeightMap);
   }

   default boolean getPublishTerrainMap()
   {
      return get(publishTerrainMap);
   }
}

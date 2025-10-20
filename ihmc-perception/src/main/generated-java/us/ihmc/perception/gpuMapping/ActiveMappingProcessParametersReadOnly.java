package us.ihmc.perception.gpuMapping;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.gpuMapping.ActiveMappingProcessParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ActiveMappingProcessParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean getRunHeightMap()
   {
      return get(runHeightMap);
   }

   default boolean getRunTerrainMap()
   {
      return get(runTerrainMap);
   }

   default boolean getRunD457TunableTransform()
   {
      return get(runD457TunableTransform);
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

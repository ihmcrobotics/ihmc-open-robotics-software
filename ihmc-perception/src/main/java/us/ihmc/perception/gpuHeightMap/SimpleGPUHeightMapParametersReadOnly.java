package us.ihmc.perception.gpuHeightMap;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapParameters.*;

public interface SimpleGPUHeightMapParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getResolution()
   {
      return get(resolution);
   }

   default double getMapLength()
   {
      return get(mapLength);
   }

   default double getMaxHeightRange()
   {
      return get(maxHeightRange);
   }

   default double getMinValidDistance()
   {
      return get(minValidDistance);
   }

   default double getRampedHeightRangeB()
   {
      return get(rampedHeightRangeB);
   }

   default double getRampedHeightRangeA()
   {
      return get(rampedHeightRangeA);
   }

   default double getRampedHeightRangeC()
   {
      return get(rampedHeightRangeC);
   }
}

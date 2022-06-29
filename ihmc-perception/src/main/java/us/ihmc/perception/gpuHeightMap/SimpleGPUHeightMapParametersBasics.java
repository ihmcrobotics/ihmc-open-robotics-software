package us.ihmc.perception.gpuHeightMap;

import us.ihmc.tools.property.StoredPropertySetBasics;

public interface SimpleGPUHeightMapParametersBasics extends SimpleGPUHeightMapParametersReadOnly, StoredPropertySetBasics
{
   default void setResolution(double resolution)
   {
      set(SimpleGPUHeightMapParameters.resolution, resolution);
   }

   default void setMapLength(double mapLength)
   {
      set(SimpleGPUHeightMapParameters.mapLength, mapLength);
   }

   default void setMaxHeightRange(double maxHeightRange)
   {
      set(SimpleGPUHeightMapParameters.maxHeightRange, maxHeightRange);
   }

   default void setMinValidDistance(double minValidDistance)
   {
      set(SimpleGPUHeightMapParameters.minValidDistance, minValidDistance);
   }

   default void setRampedHeightRangeB(double rampedHeightRangeB)
   {
      set(SimpleGPUHeightMapParameters.rampedHeightRangeB, rampedHeightRangeB);
   }

   default void setRampedHeightRangeA(double rampedHeightRangeA)
   {
      set(SimpleGPUHeightMapParameters.rampedHeightRangeA, rampedHeightRangeA);
   }

   default void setRampedHeightRangeC(double rampedHeightRangeC)
   {
      set(SimpleGPUHeightMapParameters.rampedHeightRangeC, rampedHeightRangeC);
   }
}

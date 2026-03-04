package us.ihmc.perception.gpuMapping;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.gpuMapping.TerrainMapParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface TerrainMapParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getNormalSearchRadius()
   {
      return get(normalSearchRadius);
   }

   default double getCliffSearchRadius()
   {
      return get(cliffSearchRadius);
   }

   default double getCliffHeightThreshold()
   {
      return get(cliffHeightThreshold);
   }

   default double getCliffHeightTolerance()
   {
      return get(cliffHeightTolerance);
   }

   default double getMinSupportAreaFraction()
   {
      return get(minSupportAreaFraction);
   }

   default double getMinSnapHeightThreshold()
   {
      return get(minSnapHeightThreshold);
   }

   default double getSnapHeightThresholdAtSearchEdge()
   {
      return get(snapHeightThresholdAtSearchEdge);
   }

   default double getSteppingCosineThreshold()
   {
      return get(steppingCosineThreshold);
   }

   default double getSquaredErrorThreshold()
   {
      return get(squaredErrorThreshold);
   }

   default double getBoundingBoxSizeX()
   {
      return get(boundingBoxSizeX);
   }

   default double getBoundingBoxSizeY()
   {
      return get(boundingBoxSizeY);
   }

   default double getBoundingBoxOffsetX()
   {
      return get(boundingBoxOffsetX);
   }

   default double getBoundingBoxOffsetZ()
   {
      return get(boundingBoxOffsetZ);
   }
}

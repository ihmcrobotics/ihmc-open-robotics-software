package us.ihmc.perception.rapidRegions;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.rapidRegions.RapidRegionsExtractorParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface RapidRegionsExtractorParametersReadOnly extends StoredPropertySetReadOnly
{
   default int getNormalPackRange()
   {
      return get(normalPackRange);
   }

   default int getCentroidPackRange()
   {
      return get(centroidPackRange);
   }

   default int getMergeRange()
   {
      return get(mergeRange);
   }

   default double getMergeOrthogonalThreshold()
   {
      return get(mergeOrthogonalThreshold);
   }

   default double getMergeDistanceThreshold()
   {
      return get(mergeDistanceThreshold);
   }

   default double getMergeAngularThreshold()
   {
      return get(mergeAngularThreshold);
   }

   default int getConnectionThreshold()
   {
      return get(connectionThreshold);
   }

   default int getPatchSize()
   {
      return get(patchSize);
   }

   default int getDeadPixelFilterPatchSize()
   {
      return get(deadPixelFilterPatchSize);
   }

   default double getFocalLengthXPixels()
   {
      return get(focalLengthXPixels);
   }

   default double getFocalLengthYPixels()
   {
      return get(focalLengthYPixels);
   }

   default double getPrincipalOffsetXPixels()
   {
      return get(principalOffsetXPixels);
   }

   default double getPrincipalOffsetYPixels()
   {
      return get(principalOffsetYPixels);
   }

   default boolean getUseFilteredImage()
   {
      return get(useFilteredImage);
   }

   default boolean getUseSVDNormals()
   {
      return get(useSVDNormals);
   }

   default int getSVDReductionFactor()
   {
      return get(svdReductionFactor);
   }

   default int getInternalSearchDepthLimit()
   {
      return get(internalSearchDepthLimit);
   }

   default int getBoundarySearchDepthLimit()
   {
      return get(boundarySearchDepthLimit);
   }

   default int getRegionMinPatches()
   {
      return get(regionMinPatches);
   }

   default int getBoundaryMinPatches()
   {
      return get(boundaryMinPatches);
   }

   default double getRegionGrowthFactor()
   {
      return get(regionGrowthFactor);
   }
}
